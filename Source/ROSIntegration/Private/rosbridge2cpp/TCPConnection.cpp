#include "TCPConnection.h"

#include "ROSIntegrationCore.h"

#include <iomanip>

#include <Interfaces/IPv4/IPv4Address.h>
#include <Serialization/ArrayReader.h>
#include <SocketSubsystem.h>

TCPConnection::TCPConnection(FString ip_addr, int port, bool bson_only) :
_ip_addr(ip_addr), _port(port), bson_mode(bson_only), running(true)
{
	_sock = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(
		NAME_Stream, TEXT("Rosbridge TCP client"), false);
	Thread = FRunnableThread::Create(this, TEXT("TCP Thread"), 0, TPri_BelowNormal);
}

TCPConnection::~TCPConnection()
{
	ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(_sock);
	delete Thread;
}

bool TCPConnection::SendMessage(std::string data)
{
	const uint8 *byte_msg = reinterpret_cast<const uint8*>(data.c_str());
	return SendMessage(byte_msg, data.length());
}

bool TCPConnection::SendMessage(const uint8_t *data, int32 length)
{
	if (IsHealthy())
	{
		// int32 bytes_sent = 0;
		// unsigned int total_bytes_to_send = length;
		// int32 num_tries = 0;
		// while (total_bytes_to_send > 0 && num_tries < 3)
		// {
		// 	bool SendResult = 0;
		// 	SendResult = _sock->Send(data, total_bytes_to_send, bytes_sent);
		//
		// 	if (SendResult) data += bytes_sent;
		// 	else ++num_tries;
		//
		// 	total_bytes_to_send -= bytes_sent;
		// }
		//
		// return total_bytes_to_send == 0;
		// int32 bytes = 0;
		// bool result = _sock->Send(data, length, bytes);
		// if (bytes < length) UE_LOG(LogROS, Warning, TEXT("[TCP]: Sent %d of %d bytes."), bytes, length);
		// return result;
		queue_messages.push_back(std::pair<const uint8_t *, int32>(data, length));
		return true;
	}
	return false;
}

uint32 TCPConnection::Run()
{
	auto addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	bool valid;
	addr->SetIp(*_ip_addr, valid);
	addr->SetPort(_port);

	if (!valid || !addr->IsValid())
	{
		UE_LOG(LogROS, Warning,	TEXT("[TCP]: IP address %s:%d is invalid."), *_ip_addr, _port);
		return 1;
	}

	UE_LOG(LogROS, Display, TEXT("[TCP]: Connecting..."));

	// Needs to be inside of Run since this may hang.
	if (!_sock->Connect(*addr)) return 2;

	// Setting up the receiver thread
	UE_LOG(LogROS, Display, TEXT("[TCP]: Comm Started"));

	TArray<uint8> binary_buffer;
	uint32_t buffer_size = 10 * 1024 * 1024;
	binary_buffer.Reserve(buffer_size);
	bool bson_state_read_length = true; // indicate that the receiver shall only get 4 bytes to start with
	int32_t bson_msg_length = 0;
	int32_t bson_msg_length_read = 0;

	while (running) {
		if (_sock->GetConnectionState() != ESocketConnectionState::SCS_Connected) {
			UE_LOG(LogROS, Error, TEXT("[TCP]: Error on connection"));
			ReportError(rosbridge2cpp::TransportError::R2C_SOCKET_ERROR);
			return 3;
		}

		// TX - Needs to be inside of Run otherwise our Send messages may block
		// depending on network infrastructure.
		// TODO

		// RX
		if (bson_mode)
		{
			if (bson_state_read_length)
			{
				bson_msg_length_read = 0;
				binary_buffer.SetNumUninitialized(4, false);
				int32 bytes_read = 0;
				if (_sock->Recv(binary_buffer.GetData(), 4, bytes_read) && bytes_read > 0)
				{
					bson_msg_length_read += bytes_read;
					if (bytes_read == 4) {
#if PLATFORM_LITTLE_ENDIAN
						bson_msg_length = (
							binary_buffer.GetData()[3] << 24 |
							binary_buffer.GetData()[2] << 16 |
							binary_buffer.GetData()[1] << 8 |
							binary_buffer.GetData()[0]
						);
#else
						bson_msg_length = *((uint32_t*)&binary_buffer[0]);
#endif
						// Indicate the message retrieval mode
						bson_state_read_length = false;
						binary_buffer.SetNumUninitialized(bson_msg_length, false);
					}
					else UE_LOG(LogROS, Error, TEXT("[TCP]: bytes_read is not 4 in bson_state_read_length==true. It's: %d"), bytes_read);
				}
				else
				{
					UE_LOG(LogROS, Error, TEXT("[TCP]: Failed to recv(); Closing receiver thread."));
					return 4;
				}
			}
			// Message retrieval mode
			else
			{
				int32 bytes_read = 0;
				if (_sock->Recv(binary_buffer.GetData() + bson_msg_length_read,
												bson_msg_length - bson_msg_length_read,
												bytes_read) && bytes_read > 0)
				{
					bson_msg_length_read += bytes_read;
					if (bson_msg_length_read == bson_msg_length)
					{
						// Full received message!
						bson_state_read_length = true;
						bson_t b;
						if (!bson_init_static(&b, binary_buffer.GetData(), bson_msg_length_read)) {
							UE_LOG(LogROS, Error, TEXT("[TCP]: Error on BSON parse - Ignoring message"));
							continue;
						}
						if (incoming_message_callback_bson_) incoming_message_callback_bson_(b);
					}
					else UE_LOG(LogROS, Display, TEXT("[TCP]: Binary buffer num is: %d"), binary_buffer.Num());
				}
				else UE_LOG(LogROS, Error, TEXT("[TCP]: Failed to recv()"));
			}
		}
		else {
			FString result;
			uint32 count = 0;
			while (_sock->HasPendingData(count) && count > 0) {
				FArrayReader data;
				data.SetNumUninitialized(count);

				int32 bytes_read = 0;
				// read pending data into the Data array reader
				if (_sock->Recv(data.GetData(), data.Num(), bytes_read))
				{
					int32 dest_len = TStringConvert<ANSICHAR, TCHAR>::ConvertedLength((char*)(data.GetData()), data.Num());
					UE_LOG(LogROS, VeryVerbose, TEXT("count is %d"), count);
					UE_LOG(LogROS, VeryVerbose, TEXT("bytes_read is %d"), bytes_read);
					UE_LOG(LogROS, VeryVerbose, TEXT("dest_len will be %i"), dest_len);
					TCHAR* dest = new TCHAR[dest_len + 1];
					TStringConvert<ANSICHAR, TCHAR>::Convert(dest, dest_len, (char*)(data.GetData()), data.Num());
					dest[dest_len] = '\0';

					result += dest;

					delete[] dest;
				}
				FPlatformProcess::Sleep(1);
			}
			if (result.Len() == 0) continue;

			// TODO catch parse error properly
			// auto j = json::parse(received_data);
			json j;
			j.Parse(TCHAR_TO_UTF8(*result));

			if (_incoming_message_callback) _incoming_message_callback(j);
		}
	}

	return 0;
}

void TCPConnection::Exit()
{
	running = false;
	Thread->WaitForCompletion();
	_sock->Close();
	UE_LOG(LogROS, Display, TEXT("[TCP]: Exited"));
}

void TCPConnection::Stop()
{
	running = false;
	UE_LOG(LogROS, Display, TEXT("[TCP]: Stopped"));
}

void TCPConnection::RegisterIncomingMessageCallback(std::function<void(json&)> fun)
{
	_incoming_message_callback = fun;
}

void TCPConnection::RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun)
{
	incoming_message_callback_bson_ = fun;
}

void TCPConnection::RegisterErrorCallback(std::function<void(rosbridge2cpp::TransportError)> fun)
{
	_error_callback = fun;
}

void TCPConnection::ReportError(rosbridge2cpp::TransportError err)
{
	if (_error_callback) _error_callback(err);
}

void TCPConnection::SetTransportMode(rosbridge2cpp::ITransportLayer::TransportMode mode)
{
	switch (mode) {
	case rosbridge2cpp::ITransportLayer::JSON:
		bson_mode = false;
		break;
	case rosbridge2cpp::ITransportLayer::BSON:
		bson_mode = true;
		break;
	default:
		UE_LOG(LogROS, Error, TEXT("[TCP]: Given TransportMode not implemented!"));
	}
}

bool TCPConnection::IsHealthy() const
{
	return _sock->GetConnectionState() == ESocketConnectionState::SCS_Connected && running;
}

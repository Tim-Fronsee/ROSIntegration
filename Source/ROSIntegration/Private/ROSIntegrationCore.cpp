#include "ROSIntegrationCore.h"

#include <sstream>

#include "RI/Topic.h"
#include "RI/Service.h"
#include "SpawnObjectMessage.h"

DEFINE_LOG_CATEGORY(LogROS);

#define UNREAL_ROS_CHECK_KEY_FOUND \
	if (!key_found) {\
		UE_LOG(LogROS, Warning, TEXT("%s is not present in data"), *FString(UTF8_TO_TCHAR(LookupKey.c_str())));\
		return;\
	}

UROSIntegrationCore::UROSIntegrationCore(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
	_SpawnManager = NewObject<USpawnManager>(USpawnManager::StaticClass());
	_Connection = new TCPConnection();
	_Ros = new rosbridge2cpp::ROSBridge(*_Connection);
	UE_LOG(LogROS, Display, TEXT("[ROSIntegrationCore]: Spawned"));
}

void UROSIntegrationCore::Init(FString ROSBridgeHost, int32 ROSBridgePort, bool bson_mode) {
	Stop();
	_Connection->Start(ROSBridgeHost, ROSBridgePort, bson_mode);
	_Ros->Start(bson_mode);
}

void UROSIntegrationCore::Stop()
{
	_Ros->Stop();
	_Connection->Stop();
}

bool UROSIntegrationCore::IsHealthy() const
{
	return (_Connection && _Connection->IsHealthy()) && (_Ros && _Ros->IsHealthy());
}

void UROSIntegrationCore::SetWorld(UWorld* World)
{
	_World = World;
}

void UROSIntegrationCore::InitSpawnManager()
{
	// Listen to the object spawning thread
	_SpawnMessageListener = std::unique_ptr<rosbridge2cpp::ROSTopic>(
		new rosbridge2cpp::ROSTopic(*_Ros, "/unreal_ros/spawn_objects", "visualization_msgs/Marker"));
	_SpawnMessageListener->Subscribe(std::bind(&UROSIntegrationCore::SpawnMessageCallback, this, std::placeholders::_1));

	_SpawnArrayMessageListener = std::unique_ptr<rosbridge2cpp::ROSTopic>(
		new rosbridge2cpp::ROSTopic(*_Ros, "/unreal_ros/spawn_objects_array", "visualization_msgs/MarkerArray"));
	_SpawnArrayMessageListener->Subscribe(
		std::bind(&UROSIntegrationCore::SpawnArrayMessageCallback, this, std::placeholders::_1));

	_SpawnManager->_World = _World;
	_SpawnManager->_TickingActive = true;
}

void UROSIntegrationCore::BeginDestroy()
{
	Super::BeginDestroy();
	Stop();
	delete _Connection;
	delete _Ros;
}

void UROSIntegrationCore::SpawnMessageCallback(const ROSBridgePublishMsg& message)
{
	UE_LOG(LogROS, Warning, TEXT("RECEIVED SPAWN MESSAGE --- Not implemented yet. Use the SpawnArray topic instead"));
}
void UROSIntegrationCore::SpawnArrayMessageCallback(const ROSBridgePublishMsg& message)
{
	if (!rosbridge2cpp::Helper::bson_has_key(*message.full_msg_bson_, "msg.markers")) {
		UE_LOG(LogROS, Warning, TEXT("msg.markers field missing from SpawnArray Message"));
		return;
	}

	bson_iter_t iter;
	bson_iter_t val;

	if (bson_iter_init(&iter, message.full_msg_bson_) && bson_iter_find_descendant(&iter, "msg.markers", &val) &&
		BSON_ITER_HOLDS_ARRAY(&val)) {
		UE_LOG(LogROS, Verbose, TEXT("Marker is included and is an array!"));
	} else {
		UE_LOG(LogROS, Verbose, TEXT("Marker is not included or isn't an array!"));
	}

	const char* key;
	bson_iter_t child;
	uint32_t array_len = 0;

	bson_iter_recurse(&val, &child);
	while (bson_iter_next(&child)) {
		key = bson_iter_key(&child);
		if (BSON_ITER_HOLDS_DOCUMENT(&child)) {
			array_len++;
		}
	}

	// Construct dot notation address to fetch data
	for (uint32_t i = 0; i < array_len; ++i) {
		double value;
		int32 ivalue;
		bool key_found;
		SpawnObjectMessage Message;

		const std::string marker_key_prefix("msg.markers.");
		std::string LookupKey;
		std::stringstream LookupKeyStream;

		// TODO make this more generic or use other BSON library
		// Use Templates instead of different functions?
		// Make a Map that contains a tuple of (key, typ,e reference_to_variable_in_message) to automate everything?
		LookupKeyStream << marker_key_prefix << i << ".pose.position.x";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Pose._Position.X = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".pose.position.y";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Pose._Position.Y = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".pose.position.z";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Pose._Position.Z = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".pose.orientation.x";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Pose._Orientation.X = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".pose.orientation.y";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Pose._Orientation.Y = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".pose.orientation.z";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Pose._Orientation.Z = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".pose.orientation.w";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Pose._Orientation.W = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".type";
		LookupKey = LookupKeyStream.str();
		ivalue = rosbridge2cpp::Helper::get_int32_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Type = ivalue;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".id";
		LookupKey = LookupKeyStream.str();
		ivalue = rosbridge2cpp::Helper::get_int32_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Id = ivalue;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".action";
		LookupKey = LookupKeyStream.str();
		ivalue = rosbridge2cpp::Helper::get_int32_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Action = ivalue;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".scale.x";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Scale.X = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".scale.y";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Scale.Y = value;

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".scale.z";
		LookupKey = LookupKeyStream.str();
		value = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Scale.Z = value;

		// Color
		double R, G, B, CAlpha;
		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".color.r";
		LookupKey = LookupKeyStream.str();
		R = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".color.g";
		LookupKey = LookupKeyStream.str();
		G = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".color.b";
		LookupKey = LookupKeyStream.str();
		B = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".color.a";
		LookupKey = LookupKeyStream.str();
		CAlpha = rosbridge2cpp::Helper::get_double_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND

		Message._Color = FLinearColor(R, G, B, CAlpha);

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".text";
		LookupKey = LookupKeyStream.str();
		std::string MsgText =
		rosbridge2cpp::Helper::get_utf8_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._Text = FString(MsgText.c_str());

		LookupKeyStream.str("");
		LookupKeyStream.clear();
		LookupKeyStream << marker_key_prefix << i << ".mesh_resource";
		LookupKey = LookupKeyStream.str();
		std::string MeshResource =
		rosbridge2cpp::Helper::get_utf8_by_key(LookupKey.c_str(), *message.full_msg_bson_, key_found);
		UNREAL_ROS_CHECK_KEY_FOUND
		Message._MeshResource = FString(MeshResource.c_str());

		UE_LOG(LogROS, Verbose, TEXT("Enqueue Message"));
		_SpawnManager->_SpawnObjectMessageQueue.Enqueue(Message);
		UE_LOG(LogROS, Verbose, TEXT("Enqueue Message Done"));
	}
}

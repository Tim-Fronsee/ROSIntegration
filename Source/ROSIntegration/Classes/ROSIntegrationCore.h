#pragma once

#include <CoreMinimal.h>
#include <UObject/ObjectMacros.h>
#include <UObject/Object.h>

#include "ros_bridge.h"
#include "ros_topic.h"
#include "SpawnManager.h"

ROSINTEGRATION_API DECLARE_LOG_CATEGORY_EXTERN(LogROS, Display, All);

#include "TCPConnection.h"

#include "ROSIntegrationCore.generated.h"

class TDeleterNot
{
public:
	void operator()(void*) {}
};

UCLASS()
class ROSINTEGRATION_API UROSIntegrationCore : public UObject
{
	GENERATED_UCLASS_BODY()

public:
	rosbridge2cpp::ROSBridge * _Ros;
	TCPConnection * _Connection;

	void Init(FString ROSBridgeHost, int32 ROSBridgePort, bool bson_mode);
	void Stop();

	bool IsHealthy() const;

	// You must call Init() before using this method to set upthe Implmentation correctly
	void SetWorld(UWorld* World);

	void InitSpawnManager();

	void BeginDestroy();

private:
	UPROPERTY()
	USpawnManager* _SpawnManager;

	UWorld* _World = nullptr;

	std::unique_ptr<rosbridge2cpp::ROSTopic> _SpawnMessageListener;
	std::unique_ptr<rosbridge2cpp::ROSTopic> _SpawnArrayMessageListener;

	friend class UTopic;
	friend class UService;

	void SpawnMessageCallback(const ROSBridgePublishMsg& message);
	void SpawnArrayMessageCallback(const ROSBridgePublishMsg& message);
};

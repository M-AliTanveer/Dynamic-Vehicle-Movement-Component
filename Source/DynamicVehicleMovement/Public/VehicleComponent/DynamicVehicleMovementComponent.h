// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "UObject/ObjectMacros.h"
#include "ChaosVehicleMovementComponent.h"
#include "Curves/CurveFloat.h"
#include "VehicleUtility.h"
#include "Chaos/PBDSuspensionConstraints.h"
#include "PhysicsProxy/SingleParticlePhysicsProxyFwd.h"
#include "Engine/EngineTypes.h"
#include "VehicleStructsAndEnums.h"
#include "Components/AudioComponent.h"
#include "DynamicVehicleMovementComponent.generated.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

class DYNAMICVEHICLEMOVEMENT_API UDynamicVehicleSimulation : public UChaosVehicleSimulation
{
public:

	UDynamicVehicleSimulation(): bOverlapHit(false)
	{
		QueryBox.Init();
	}

	virtual ~UDynamicVehicleSimulation()
	{
	}

	virtual void Init(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicleIn) override
	{
		UChaosVehicleSimulation::Init(PVehicleIn);

		WheelState.Init(PVehicle->Wheels.Num());

		dataImpactingSimulation = FDynamicSimulationData();
	}

	virtual void UpdateConstraintHandles(TArray<FPhysicsConstraintHandle>& ConstraintHandlesIn) override;

	virtual void TickVehicle(UWorld* WorldIn, float DeltaTime, const FChaosVehicleAsyncInput& InputData, FChaosVehicleAsyncOutput& OutputData, Chaos::FRigidBodyHandle_Internal* Handle) override;

	/** Advance the vehicle simulation */
	virtual void UpdateSimulation(float DeltaTime, const FChaosVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle) override;

	/** Update the vehicle state */
	virtual void UpdateState(float DeltaTime, const FChaosVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle) override;

	virtual void FillOutputState(FChaosVehicleAsyncOutput& Output) override;

	/** Are enough vehicle systems specified such that physics vehicle simulation is possible */
	virtual bool CanSimulate() const override;

	/** Pass control Input to the vehicle systems */
	virtual void ApplyInput(const FControlInputs& ControlInputs, float DeltaTime) override;

	/** Perform suspension ray/shape traces */
	virtual void PerformSuspensionTraces(const TArray<Chaos::FSuspensionTrace>& SuspensionTrace, FCollisionQueryParams& TraceParams, FCollisionResponseContainer& CollisionResponse, TArray<FWheelTraceParams>& WheelTraceParams);


	/** Update the engine/transmission simulation */
	virtual void ProcessMechanicalSimulation(float DeltaTime);

	/** Process steering mechanism */
	virtual void ProcessSteering(const FControlInputs& ControlInputs);

	/** calculate and apply lateral and longitudinal friction forces from wheels */
	virtual void ApplyWheelFrictionForces(float DeltaTime);

	/** calculate and apply chassis suspension forces */
	virtual void ApplySuspensionForces(float DeltaTime, TArray<FWheelTraceParams>& WheelTraceParams);

	bool IsWheelSpinning() const;
	bool ContainsTraces(const FBox& Box, const TArray<struct Chaos::FSuspensionTrace>& SuspensionTrace);

	/** Draw 3D debug lines and things along side the 3D model */
	virtual void DrawDebug3D() override;

	float GetRelativeEngineRPM_FromSpeed(float currentSpeed = 0);

	FDynamicWheelState WheelState;	/** Cached state that holds wheel data for this frame */

	TArray<FPhysicsConstraintHandle> ConstraintHandles;

	FDynamicSimulationData dataImpactingSimulation;

	// cache trace overlap query
	TArray<FOverlapResult> OverlapResults;
	bool bOverlapHit;
	FBox QueryBox;
	float targetRPM_BasedOnFuel = 0;

private:
	float prevSpeed = 0;
};

/**
 * 
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent), hidecategories = (PlanarMovement, "Components|Movement|Planar", Activation, "Components|Activation"))
class DYNAMICVEHICLEMOVEMENT_API UDynamicVehicleMovementComponent : public UChaosVehicleMovementComponent
{
	GENERATED_BODY()

public:
	UDynamicVehicleMovementComponent(const FObjectInitializer& ObjectInitializer);

	// Called every frame
	void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction);

	virtual void BeginPlay() override;

	UPROPERTY(BlueprintAssignable)
	FOnVehicleActionError actionErrorCaused;

	UPROPERTY(BlueprintAssignable)
	FOnVehicleStateChanged vehicleStateChanged;

	UPROPERTY(BlueprintAssignable)
	FOnVehicleGearChanged gearChanged;

	UPROPERTY(BlueprintAssignable)
	FOnVehicleTransmissionSystemChanged transmissionSystemChanged;

	UPROPERTY(BlueprintAssignable)
	FOnVehicleStarted vehicleStarted;

	UPROPERTY(BlueprintAssignable)
	FOnVehicleStopped vehicleStopped;

	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChanged gasInputChanged; 
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChanged clutchInputChanged; 
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChanged manualFuelHandleInputChanged; 
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChanged brakeInputChanged;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChanged wiperInputChanged;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChangedBool handbrakeInputChanged;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChangedBool brakeAssistInputChanged;

	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased gasInputReleased; 
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased clutchInputReleased; 
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased manualFuelHandleInputReleased; 
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased brakeInputReleased;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased wiperInputReleased;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased handbrakeInputReleased;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased brakeAssistInputReleased;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChangedBool headLightsInputChanged;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased headLightsInputReleased;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChangedBool highBeamLightsInputChanged;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased highBeamLightsInputReleased;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputChangedBool fogLightsInputChanged;
	UPROPERTY(BlueprintAssignable)
	FOnVehicleInputReleased fogLightsInputReleased;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	bool bSuspensionEnabled;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	bool bWheelFrictionEnabled;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	bool bLegacyWheelFrictionPosition;

	/** Wheels to create */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	TArray<FDynamicWheelSetup> WheelSetupsForDifferentialSystem1;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	TArray<FDynamicWheelSetup> WheelSetupsForDifferentialSystem2;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	struct FCollisionResponseContainer WheelTraceCollisionResponses;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement")
	bool bMechanicalSimEnabled;

	/** Engine */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (EditCondition = "bMechanicalSimEnabled"))
	FDynamicVehicleEngineConfig EngineSetup;

	/** Differential */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (EditCondition = "bMechanicalSimEnabled"))
	FDynamicVehicleDifferentialConfig DifferentialSetup;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (EditCondition = "bMechanicalSimEnabled"))
	FDynamicVehicleTransmissionConfig TransmissionSetup;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement")
	FDynamicVehicleSteeringConfig SteeringSetup;

	// Our instanced wheels
	UPROPERTY(transient, duplicatetransient, BlueprintReadOnly, Category = Vehicle)
	TArray<TObjectPtr<class UDynamicWheel>> Wheels;

	/** Get current engine's rotation speed */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	float GetEngineRotationSpeed();

	/** Get current engine's max rotation speed */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	float GetEngineMaxRotationSpeed() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	int GetNumWheels() const
	{
		return WheelStatus.Num();
	}

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheelStatus(const struct FDynamicWheelStatus& Status, bool& bInContact, FVector& ContactPoint, UPhysicalMaterial*& PhysMaterial
		, float& NormalizedSuspensionLength, float& SpringForce, float& SlipAngle, bool& bIsSlipping, float& SlipMagnitude, bool& bIsSkidding, float& SkidMagnitude, FVector& SkidNormal, float& DriveTorque, float& BrakeTorque, bool& bABSActivated);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FDynamicWheelStatus MakeWheelStatus(bool bInContact, FVector& ContactPoint, UPhysicalMaterial* PhysMaterial
		, float NormalizedSuspensionLength, float SpringForce, float SlipAngle, bool bIsSlipping, float SlipMagnitude, bool bIsSkidding, float SkidMagnitude, FVector& SkidNormal, float DriveTorque, float BrakeTorque, bool bABSActivated);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheeledSnapshot(const struct FWheeledSnaphotData& Snapshot, FTransform& Transform, FVector& LinearVelocity
		, FVector& AngularVelocity, int& SelectedGear, float& EngineRPM, TArray<FWheelSnapshot>& WheelSnapshots);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FWheeledSnaphotData MakeWheeledSnapshot(FTransform Transform, FVector LinearVelocity, FVector AngularVelocity
		, int SelectedGear, float EngineRPM, const TArray<FWheelSnapshot>& WheelSnapshots);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheelSnapshot(const struct FWheelSnapshot& Snapshot, float& SuspensionOffset
		, float& WheelRotationAngle, float& SteeringAngle, float& WheelRadius, float& WheelAngularVelocity);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FWheelSnapshot MakeWheelSnapshot(float SuspensionOffset, float WheelRotationAngle
		, float SteeringAngle, float WheelRadius, float WheelAngularVelocity);

	/** Get a wheels current simulation state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	const FDynamicWheelStatus& GetWheelState(int WheelIndex) const
	{
		return WheelStatus[WheelIndex];
	}

	virtual float GetSuspensionOffset(int WheelIndex) override;
	UPhysicalMaterial* GetPhysMaterial(int WheelIndex);

	/** Set all channels to the specified response - for wheel raycasts */
	void SetWheelTraceAllChannels(ECollisionResponse NewResponse)
	{
		WheelTraceCollisionResponses.SetAllChannels(NewResponse);
	}

	/** Set the response of this body to the supplied settings - for wheel raycasts */
	void SetWheelTraceResponseToChannel(ECollisionChannel Channel, ECollisionResponse NewResponse)
	{
		WheelTraceCollisionResponses.SetResponse(Channel, NewResponse);
	}

	/** Get Collision ResponseToChannels container for this component **/
//	FORCEINLINE_DEBUGGABLE const FCollisionResponseContainer& GetTraceResponseToChannels() const { return WheelTraceCollisionResponses.GetResponseContainer(); }

	//////////////////////////////////////////////////////////////////////////
	// Public

	virtual void Serialize(FArchive& Ar) override;
	virtual void PostLoad() override;

	// Get output data from Physics Thread
	virtual void ParallelUpdate(float DeltaSeconds);

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	/** Are the configuration references configured sufficiently that the vehicle can be created */
	virtual bool CanCreateVehicle() const override;

	/** Used to create any physics engine information for this component */
	virtual void OnCreatePhysicsState() override;

	/** Used to shut down and pysics engine structure for this component */
	virtual void OnDestroyPhysicsState() override;

	/** display next debug page */
	static void NextDebugPage();

	/** display previous debug page */
	static void PrevDebugPage();

	/** Enable or completely bypass the ProcessMechanicalSimulation call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void EnableMechanicalSim(bool InState)
	{
		bMechanicalSimEnabled = InState;
	}

	/** Enable or completely bypass the ApplySuspensionForces call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void EnableSuspension(bool InState)
	{
		bSuspensionEnabled = InState;
	}

	/** Enable or completely bypass the ApplyWheelFrictionForces call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void EnableWheelFriction(bool InState)
	{
		bWheelFrictionEnabled = InState;
	}

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelClass(int WheelIndex, TSubclassOf<UDynamicWheel> InWheelClass);

	/** Grab a snapshot of the vehicle instance dynamic state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	virtual FWheeledSnaphotData GetSnapshot() const;

	/** Set snapshot of vehicle instance dynamic state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	virtual void SetSnapshot(const FWheeledSnaphotData& SnapshotIn);


	//////////////////////////////////////////////////////////////////////////
	// change handling via blueprint at runtime
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetMaxEngineTorque(float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetDragCoefficient(float DragCoeff);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetDownforceCoefficient(float DownforceCoeff);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetDifferentialFrontRearSplit(float FrontRearSlpit);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetTractionControlEnabled(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetABSEnabled(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetAffectedByBrake(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetAffectedByHandbrake(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetAffectedBySteering(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetAffectedByEngine(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelRadius(int WheelIndex, float Radius);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelFrictionMultiplier(int WheelIndex, float Friction);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelSlipGraphMultiplier(int WheelIndex, float Multiplier);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelMaxBrakeTorque(int WheelIndex, float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelHandbrakeTorque(int WheelIndex, float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelMaxSteerAngle(int WheelIndex, float AngleDegrees);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetTorqueCombineMethod(ETorqueCombineMethod InCombineMethod, int32 WheelIndex);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetDriveTorque(float DriveTorque, int32 WheelIndex);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetBrakeTorque(float BrakeTorque, int32 WheelIndex);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetSuspensionParams(float Rate, float Damping, float Preload, float MaxRaise, float MaxDrop, int32 WheelIndex);

	/** */
	virtual TUniquePtr<Chaos::FSimpleWheeledVehicle> CreatePhysicsVehicle() override
	{
		// Make the Vehicle Simulation class that will be updated from the physics thread async callback
		VehicleSimulationPT = MakeUnique<UDynamicVehicleSimulation>();

		return UChaosVehicleMovementComponent::CreatePhysicsVehicle();
	}

	/** Allocate and setup the Chaos vehicle */
	virtual void SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle) override;

	virtual void ResetVehicleState() override;



	//CUSTOMIZED FUNCTIONALITY FUNCTIONS START HERE

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Differential System", BlueprintPure)
	//Returns the current in use Differenital System.
	bool IsUsingSystem1ForDifferential() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Differential System", BlueprintPure)
	//Returns wether differential system can be changed.
	bool CanChangeDifferentialSystem();	

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System", BlueprintPure)
	//Returns wether high ratios are in use or low
	bool IsUsingHighGears() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns wether engine is started or not
	bool IsEngineStarted() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns wether vehicle is accelerating or decelerating
	bool IsVehicleCurrentlyAccelerating() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns current Engine State.
	EEngineState GetCurrentEngineState() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns wether vehicle is automatic or manual
	bool IsVehicleUsingAutomaticTransmission() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Input", BlueprintPure)
	//Returns current vehicle input structure
	FDynamicInputData GetCurrentInputData() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transfer Case System", BlueprintPure)
	//Returns wether differential system can be changed.
	bool CanChangeTransferCasePosition();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System", BlueprintPure)
	//Returns Current Gear. Should be used instead of GetCurrentGear function.
	int GetCurrentActiveGear() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System", BlueprintPure)
	//Returns Current Gear's ratio. Takes into account high/low gear switch. Returns with final ratio and transfer case ratio multiplied
	float GetCurrentActiveGearRatio() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System", BlueprintPure)
	//Final Gear ratio is multiplied with each individual gear's ratio. This eliminates that effect and returns true gear ratio.
	float GetCurrentActiveGearRatioWithoutFinalGearRatioAffect() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns speed in Kilometers Per hour
	float GetVehicleSpeedInKM_PerHour() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns Wheel RPMs
	TArray<float> GetWheelSpeeds() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns Enigne Torque
	float GetEngineTorque() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Miscellanous", BlueprintPure)
	//Returns vehicle center of mass. Center of mass affects handling a lot. Use this to debug.
	FTransform GetCenterOfMass(bool local = true);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Miscellanous", BlueprintPure)
	//Returns net fuel intake taking into account manual fuel handle and gas input. Base implementation only returns greater value of the two.
	//If input != -1, then input will be used to evaluate instead of gas input
	virtual float GetNetFuelIntake(float inputGasValue = -1.0f);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Functionalities", BlueprintPure)
	//Get structure representing active vehicle functionalities
	FDynamicFunctionalities GetActiveVehicleFunctionalities() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transfer Case System", BlueprintPure)
	//Get Transfer Case configuration
	FDyamicTransferCaseConfig GetTransferCaseConfig() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns whether any form of break is active?
	bool IsBreakActiveInAnyForm(bool overrideValue = false) const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns true if in reverse, else false
	bool IsReversing();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns traveled distance if tracking is on, else returns 0
	float GetTraveledDistance();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	bool IsVehicleOnSlope(float &outSlopeAngle, float AngleThreshold = 0 );


	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	ESlideDirection GetSlideDirection();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	EVehicleMovementDirection GetMovementDirection();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Lights", BlueprintPure)
	bool AreLightsOn(bool checkFogLights);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	bool IsGasPedalPressed(float aboveValue = 0);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//enables or disables distance tracking;
	void ToggleTrackingTravelDistance(bool enable);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Lights")
	//Toggle Lights like headlights, rear lights. 
	//Does not influence break and turn lights
	void ToggleLights(bool enable);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Lights")
	//Toggle Lights like headlights, rear lights. 
	//Does not influence break and turn lights
	void ToggleFogLights(bool enable, bool front);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Lights")
	void ToggleHighbeam(bool enable, bool overRideOnSwitchForLight = false);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Lights")
	//Handles Indicator flickering every .5 seconds
	void ToggleIndicators(bool enable, bool left);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Lights")
	//Only turns on and off light based on input. Use if you want to drive indicators manually
	void ToggleIndicatorsManually(bool enable, bool left);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Sounds")
	void ToggleVehicleHorn(bool enable, bool low);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Differential System") 
	//Set New Active Differential System. True to activate system 1, false for system 2. Truck needs to be in neutral and rest. Will return true if system changed. You can use failure reason for deubg.
	bool SetActiveSystemForDifferential(bool UseSystem1, FString& failureReason);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transfer Case System")
	//Set New Ratio of Transfer Case if enabled.
	bool SetTransferCasePosition(ETransferCasePosition newPosition, FString &failureReason);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transfer Case System")
	//Set New Ratio of Transfer Case if enabled.
	bool SetTransferCasePositionUsingNum(int newPosition, FString& failureReason);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System") 
	//Change between using High Gear Ratios or Low Gear Ratios. True for using High Gear. False for Low Gear. Guaranteed Success if enabled
	void ChangeTransmissionSystem(bool useHighGears);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	/** Set the user input for gear accounting for high, low gear. It is suggested to use the wrappers GearUp, GearDown, GoNeutral instead of this one  (-1 reverse, 0 neutral, 1+ forward)*/
	bool SetNewGear(int GearNum, bool changeImmediately=true);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	//Shift to next gear if available
	void GearUp();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	//Shift to previous gear if available. 
	void GearDown();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	//Shift to neutral directly.
	void GoNeutral();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	//Shift to reverse gear directly
	void PutInReverse();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Applies Throttle Input. Use this in Both Forward Movement and Reverse Movement since Gas Pedal works for both
	bool ApplyGas(float gasPedalValue); 

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Use for Break Pedal Input
	bool ApplyBrakes(float breakPedalValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Applies Steering input.
	bool SteerVehicle(float steeringWheelValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Applies Clutch input. Useless for automatic vehicle
	bool ApplyClutch(float clutchPedalValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Applies handbreak instantly or disables it.
	bool ChangeHandBrakeState(bool handBreakActive);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Turns engine on/off. Equivalent to Ignition.
	bool SetEngineStarterValue(bool starterValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Updates Engine State based on variable values. This is the main brain of the manual vehicle.
	//Called Internally wherever needed. Only call it if you think an action should update engine state, but is missing it somehow.
	void UpdateVehicleEngineState();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Toggles Break Assist. Used in heavy vehicles
	bool ToggleBreakAssist(bool enableBreakAssist);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Use for Manual Fuel Handle Input. Call only when value changes
	bool AdjustFuelHandle(float fuelValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Non Functional Aspects")
	void SetWiperMode(EWiperModes newMode);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Non Functional Aspects")
	bool PlayRoadSound(USoundBase* newSound, bool play, float volume = 1);


#if WITH_EDITOR
	virtual bool CanEditChange(const FProperty* InProperty) const override;
#endif
	//CUSTOMIZED FUNCTIONS END HERE

	

protected:

	//////////////////////////////////////////////////////////////////////////
	// Setup

	/** Re-Compute any runtime constants values that rely on setup data */
	virtual void ComputeConstants() override;

	/** Skeletal mesh needs some special handling in the vehicle case */
	virtual void FixupSkeletalMesh();

	/** Create and setup the Chaos vehicle */
	virtual void CreateVehicle() override;

	/** Instantiate and setup our wheel objects */
	virtual void CreateWheels();

	/** Release our wheel objects */
	virtual void DestroyWheels();

	/** Set up the chassis and wheel shapes */
	virtual void SetupVehicleShapes();

	/** Setup calculated suspension parameters */
	void SetupSuspension(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle);

	/** Maps UDynamicWheel Axle to a wheel index */
	void RecalculateAxles();

	/** Get the local position of the wheel at rest */
	virtual FVector GetWheelRestingPosition(const FDynamicWheelSetup& WheelSetup);

	//////////////////////////////////////////////////////////////////////////
	// Update
	void FillWheelOutputState();

	/* Fill Async input state */
	virtual void Update(float DeltaTime) override;

	//////////////////////////////////////////////////////////////////////////
	// Debug

	/** Draw 2D debug text graphs on UI for the wheels, suspension and other systems */
	/*virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos);*/

	/** Get distances between wheels - primarily a debug display helper */
	const FVector2D& GetWheelLayoutDimensions() const
	{
		return WheelTrackDimensions;
	}

private:
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Is Using System 1 For Differential?"))
	//True for System 1 in use, False for System 2. Default is System 2.
	bool useSystem1ForDifferential = false; 
	UPROPERTY()
	//True for High gear ratios being used. False for low gears. Default is High Gears
	bool usingHighGears = true; 
	UPROPERTY()
	//True for Engine Started. False for Engine off
	bool currentEngineStartedValue = false; 
	UPROPERTY()
	//True for accelerating, False for decelrating. May behave oddly at speed = 0.
	bool isVehicleAccelerating = false; 
	UPROPERTY()
	//Engine State values. Alters between varios states
	EEngineState currentEngineState = EEngineState::EngineOff;  
	UPROPERTY(BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Transfer Case System", meta = (DisplayName = "Current InUse Transfer Case Ratio", AllowPrivateAccess = "true"))
	//Ratio being applied by the transfer case system.
	float currentTransferCaseRatio = .917;
	UPROPERTY(BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Movement", meta = (DisplayName = "Can Engine Jump Start", AllowPrivateAccess = "true"))
	//Whether vehicle meets engine jump start requriements currently
	bool canEngineJumpStart = false;
	UPROPERTY()
	FDynamicInputData currentInputs;
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Vehicle has automatic transmission?", DisplayAfter = "bMechanicalSimEnabled"))
	//True = Automatic Car. False = Manual Car
	bool isVehicleAutomatic = false;
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Vehicle Optional Functionalities", DisplayAfter = "bMechanicalSimEnabled"))
	FDynamicFunctionalities vehicleFunctionalities;
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Vehicle Optional Non Functional Aspects", DisplayAfter = "bMechanicalSimEnabled"))
	FDynamicNonFunctionalVehicleAspects vehicleNonFunctionalAspects;
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Transfer Case System", DisplayAfter = "vehicleFunctionalities"))
	FDyamicTransferCaseConfig transferCaseConfig;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Vehicle Lights", AllowPrivateAccess = "true"))
	FDynamicVehicleLights vehicleLights;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Vehicle Sounds", AllowPrivateAccess = "true"))
	FDynamicVehicleSounds vehicleSounds;
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Defaults", meta = (DisplayName = "Edit Default Input Ranges"))
	//Allows to edit default input ranges.
	bool editDefaultRanges = false;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Transmission System", meta = (DisplayName = "Clutch Threshold Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges&&isVehicleAutomatic==false"))
	//The threshold value at and after which engine allows gear change. 
	float clutchThresholdValue = 70; 
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Transmission System", meta = (DisplayName = "Clutch Disengagment Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges&&isVehicleAutomatic==false"))
	//The value at and after which engine is disengaged from wheels. May cause unexpected behaviour if changed at runtime
	float clutchDisengagementValue = 95; 
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Gas Input Minimum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Minimum possible value for gas pedal input
	float gasPedalMinValue = 0;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Gas Input Maximum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Maximum possible value for gas pedal input
	float gasPedalMaxValue = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Break Input Minimum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Minimum possible value for break pedal input
	float breakPedalMinValue = 0;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Break Input Maximum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Maximum possible value for break pedal input
	float breakPedalMaxValue = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Steering Input Minimum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Minimum possible value for steer wheel input. Minimum value = Complete Left Turn
	float steerWheelMinValue = 0;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Steering Input Maximum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Maximum possible value for steer wheel input. Maximum value = Complete Right Turn. 
	float steerWheelMaxValue = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Transmission System", meta = (DisplayName = "Clutch Input Minimum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges&&isVehicleAutomatic==false"))
	//Minimum possible value for clutch pedal input. 
	float clutchPedalMinValue = 0;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Transmission System", meta = (DisplayName = "Clutch Input Maximum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges&&isVehicleAutomatic==false"))
	//Maximum possible value for clutch pedal input.  
	float clutchPedalMaxValue = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Minimum value of Fuel Intake to sustain Idle RPM", AllowPrivateAccess = "true", EditCondition = "editDefaultRanges&&isVehicleAutomatic==false"))
	//The threshold value at which engine can sustain idle RPM. Includes Net Intake from Gas Pedal and Manual Handle. 
	//See GetNetFuelIntake() for more info
	float fuelValueToSustainIdleRPM = 30;
	UPROPERTY(BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Movement", meta = (DisplayName = "Tracked Travel Distance", AllowPrivateAccess = "true"))
	float TotalDistanceTraveled = 0;
	UPROPERTY()
	//Engine power transmission ratio to transmission system. Essentially how much power is transmitted to gears. 1 = Full (clutch not pressed) , 0 = None (clutch fully pressed)
	float currentTransmissionRatio = 0;
	UPROPERTY(BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Transmission", meta = (DisplayName = "Previous Gear", AllowPrivateAccess = "true"))
	int previousGear = 0;

	UPROPERTY()
	TObjectPtr<UAudioComponent> currentEngineSound;
	UPROPERTY()
	TObjectPtr<UAudioComponent> currentIndicatorSound;
	UPROPERTY()
	TObjectPtr<UAudioComponent> currentHornSound;
	UPROPERTY()
	TObjectPtr<UAudioComponent> currentWiperSound;
	UPROPERTY()
	TObjectPtr<UAudioComponent> currentBreakSound;
	UPROPERTY()
	TObjectPtr<UAudioComponent> currentRoadSound;

	bool PlayNewAudio(USoundBase* newSound);
	bool PlayHorn(USoundBase* newSound, bool play, float volume = 1);
	bool PlayIndicatorSound(USoundBase* newSound, bool play, float volume = 1);
	bool PlayWiperSound(USoundBase* newSound, bool play, float volume = 1);
	bool PlayBrakeSound(USoundBase* newSound, bool play, float volume = 1);

	UFUNCTION()
	void FlickerIndicator(bool leftLight);
	UFUNCTION()
	void StartFakeRPM();
	UFUNCTION()
	void EndFakeRPM();
	UFUNCTION()
	void ModifyFakeRPM();

	float fakeRPM = 0;
	bool useDelayedRPM = false;
	bool syncRPM = false;

	FTimerHandle rightIndicatorFlickerHandle, leftIndicatorFlickerHandle, decayedRPMSetter, decayedRPM;
	bool rightLightFlickerOn = false, leftLightFlickerOn = false;

	bool isOnHighBeamMode = false;
	bool shouldTrackTravelDistance = false; 
	FVector previousPosition; //for distance tracking
	float previousEngineRPM; //caches last engine RPM values. Used for calculation of state
	float previousVehicleSpeed = 0; //caches last vehicle speed
	APlayerController* playerController;
	//Do Once variables
	bool doOnceForLockedAxelError = true; //true means allow.
	bool doOnceForParkingBreakError = true; //true means allow.
	bool doOnceForEnableSlideConsoleCommand = true;
	bool doOnceForDisableSlideConsoleCommand = true;
	UPROPERTY()
	bool isDownshift = false;

	UDynamicVehicleSimulation* derivedPtrForSimulationClass ;

	/** Get distances between wheels - primarily a debug display helper */
	FVector2D CalculateWheelLayoutDimensions();


#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	float CalcDialAngle(float CurrentValue, float MaxValue);
	void DrawDial(UCanvas* Canvas, FVector2D Pos, float Radius, float CurrentValue, float MaxValue);
#endif

	struct FCachedState
	{
		FCachedState() : WheelOffset(0.f), bIsValid(false)
		{ }

		float WheelOffset;
		bool bIsValid;
	};

	EDynamicDebugPages DebugPage;
	uint32 NumDrivenWheels; /** The number of wheels that have engine enabled checked */
	FVector2D WheelTrackDimensions;	// Wheelbase (X) and track (Y) dimensions
	TMap<UDynamicWheel*, TArray<int>> AxleToWheelMap;
	TArray<FPhysicsConstraintHandle> ConstraintHandles;
	TArray<FDynamicWheelStatus> WheelStatus; /** Wheel output status */
	TArray<FCachedState> CachedState;
	Chaos::FPerformanceMeasure PerformanceMeasure;

};

//////////////////////////////////////////////////////////////////////////

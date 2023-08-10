#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "TautRopeSimulatorCPU.generated.h"

UCLASS()
// ANiagaraActorを参考にしている
class ATautRopeSimulatorCPU : public AActor
{
	GENERATED_BODY()

public:
	ATautRopeSimulatorCPU();

	virtual void PostRegisterAllComponents() override;

	virtual void BeginPlay() override;
	virtual void Tick( float DeltaSeconds ) override;

	/** Set true for this actor to self-destruct when the Niagara system finishes, false otherwise */
	UFUNCTION(BlueprintCallable)
	void SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish);

private:
	/** Pointer to System component */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UNiagaraComponent* NiagaraComponent;

#if WITH_EDITORONLY_DATA
	// Reference to sprite visualization component
	UPROPERTY()
	class UBillboardComponent* SpriteComponent;

	// Reference to arrow visualization component
	UPROPERTY()
	class UArrowComponent* ArrowComponent;

#endif

private:
	/** True for this actor to self-destruct when the Niagara system finishes, false otherwise */
	UPROPERTY()
	uint32 bDestroyOnSystemFinish : 1;

	/** Callback when Niagara system finishes. */
	UFUNCTION(CallInEditor)
	void OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent);

private:
	UPROPERTY(EditAnywhere)
	bool bDrawCollisionEdge = false;

	UPROPERTY(EditAnywhere)
	bool bDrawTraceToRemove = false;

	UPROPERTY(EditAnywhere)
	bool bDrawTensionDir = false;

	UPROPERTY(EditAnywhere, Meta = (UIMin = 0.00000002, ClampMin = 0.00000002)) // 0.00000002 is UE_SMALL_NUMBER * 2
	float Tolerance = 0.1f;

	UPROPERTY(EditAnywhere)
	float RopeRadius = 1.0f;

	UPROPERTY(EditAnywhere)
	FBox OverlapQueryBox = FBox(FVector(-500.0f, -500.0f, -200.0f), FVector(500.0f, 500.0f, 200.0f));

	UPROPERTY(EditAnywhere)
	TArray<TEnumAsByte<EObjectTypeQuery>> OverlapQueryObjectTypes;

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	class AActor* StartConstraintActor = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	class USceneComponent* StartConstraintComponent = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName StartConstraintSocket;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	AActor* EndConstraintActor = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	class USceneComponent* EndConstraintComponent = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName EndConstraintSocket;

private:
	UPROPERTY(EditAnywhere)
	int32 MaxIteration = 50;

private:
	void UpdateStartEndConstraint();
	void UpdateRopeBlockers();
	void SolveRopeBlockersCollisionConstraintOld();
	void SolveRopeBlockersCollisionConstraint();

private:
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<FVector> ParentPositions;
	TArray<FVector> ChildPositions;
	FTransform InvActorTransform = FTransform::Identity;
	float ToleranceSquared = 0.01f;

	//TODO: UPROPERTYをつけるとUHTが通らないのであきらめる
	TArray<TPair<FVector, FVector>> RopeBlockerTriMeshEdgeArray;
	TArray<int32> EdgeIdxOfPositions;
	TArray<FVector> EdgeIdxDebugDrawPositions;
	TArray<class UPrimitiveComponent*> OverlapPrimitives; // UPrimitiveComponent::LineTraceComponent()がconstでないのでしょうがなくconstをつけてない

	TArray<bool> MovedFlagOfPositions;

public:
	/** Returns NiagaraComponent subobject **/
	class UNiagaraComponent* GetNiagaraComponent() const { return NiagaraComponent; }
#if WITH_EDITORONLY_DATA
	/** Returns SpriteComponent subobject **/
	class UBillboardComponent* GetSpriteComponent() const { return SpriteComponent; }
	/** Returns ArrowComponent subobject **/
	class UArrowComponent* GetArrowComponent() const { return ArrowComponent; }
#endif

#if WITH_EDITOR
	// AActor interface
	virtual bool GetReferencedContentObjects(TArray<UObject*>& Objects) const override;
	// End of AActor interface

	/** Reset this actor in the level.*/
	void ResetInLevel();
#endif // WITH_EDITOR
};


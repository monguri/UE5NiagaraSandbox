#include "TautRopeSimulatorCPU.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Texture2D.h"
#include "Components/ArrowComponent.h"
#include "Components/BillboardComponent.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "Async/ParallelFor.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "DrawDebugHelpers.h"

void ATautRopeSimulatorCPU::PreInitializeComponents()
{
	Super::PreInitializeComponents();

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();
	ToleranceSquared = Tolerance * Tolerance; // TODO:リアルタイムには更新しない

	// まずは一つの線分から
	NumParticles = 2;
	NumSegments = NumParticles - 1;

	Positions.SetNum(NumParticles);
	PrevPositions.SetNum(NumParticles);
	ParentPositions.SetNum(NumSegments);
	ChildPositions.SetNum(NumSegments);
	EdgeIdxOfPositions.SetNum(NumParticles);

	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ParticleIdx++)
	{
		PrevPositions[ParticleIdx] = PrevPositions[ParticleIdx] = Positions[ParticleIdx] = FVector::XAxisVector * 100.0f * ParticleIdx; // 1mの長さの線分
		EdgeIdxOfPositions[ParticleIdx] = INDEX_NONE;
#if 0
		if (ParticleIdx % 2 == 0)
		{
			Orientations[ParticleIdx] = InitialOrientations[ParticleIdx] = FQuat(FVector::ZAxisVector, PI / 2) * FQuat(FVector::XAxisVector, PI / 2);
		}
		else
		{
			Orientations[ParticleIdx] = InitialOrientations[ParticleIdx] = FQuat(FVector::XAxisVector, PI / 2);
		}
#endif
	}

	for (int32 ParticleIdx = 0; ParticleIdx < NumSegments; ParticleIdx++)
	{
		ParentPositions[ParticleIdx] = Positions[ParticleIdx];
		ChildPositions[ParticleIdx] = Positions[ParticleIdx + 1];
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する

	// パーティクル数でなくセグメント数をNumParticlesには設定する
	NiagaraComponent->SetNiagaraVariableInt("NumSegments", NumSegments);

	NiagaraComponent->SetNiagaraVariableFloat("RopeRadius", RopeRadius);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ParentPositions"), ParentPositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ChildPositions"), ChildPositions);
}

void ATautRopeSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ParticleIdx++)
		{
			PrevPositions[ParticleIdx] = Positions[ParticleIdx];
		}

		UpdateStartEndConstraint();
		UpdateRopeBlockers();
		SolveRopeBlockersCollisionConstraint();
	}

	check(NumParticles >= 2);
	check(NumSegments >= 1);
#if 1
	// 直接サイズとインデックス指定でコピーする方法がないので一旦別変数にコピーしてMove。
	TArray<FVector> TmpParentPositions(Positions.GetData(), NumSegments);
	TArray<FVector> TmpChildPositions(&Positions.GetData()[1], NumSegments);
	ParentPositions = MoveTemp(TmpParentPositions);
	ChildPositions = MoveTemp(TmpChildPositions);
#else
	ParentPositions.SetNum(NumSegments);
	ChildPositions.SetNum(NumSegments);
	for (int32 ParticleIdx = 0; ParticleIdx < NumSegments; ParticleIdx++)
	{
		ParentPositions[ParticleIdx] = Positions[ParticleIdx];
		ChildPositions[ParticleIdx] = Positions[ParticleIdx + 1];
	}
#endif
	NiagaraComponent->SetNiagaraVariableInt("NumSegments", NumSegments);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ParentPositions"), ParentPositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ChildPositions"), ChildPositions);

#if 0
	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
	{
		// 初期配置がローカル座標でZ軸負方向にチェインがのびていて、そのときに回転が0という前提。
		// 子供と自分の位置差分ベクトルをパーティクルの向きとする。
		if (ParticleIdx == NumParticles - 1)
		{
			// TODO:ダミーパーティクルはやらず親と自分の位置差分にするので親のパーティクルと向きが同じになる
			// という問題は残っている
			const FVector& ParentToSelf = Positions[ParticleIdx] - Positions[ParticleIdx - 1];
			Orientations[ParticleIdx] = FQuat::FindBetween(-FVector::ZAxisVector, ParentToSelf) * InitialOrientations[ParticleIdx];
		}
		else
		{
			const FVector& SelfToChild = Positions[ParticleIdx + 1] - Positions[ParticleIdx];
			Orientations[ParticleIdx] = FQuat::FindBetween(-FVector::ZAxisVector, SelfToChild) * InitialOrientations[ParticleIdx];
		}
	}

	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayQuat(NiagaraComponent, FName("Orientations"), Orientations);
#endif
}

void ATautRopeSimulatorCPU::UpdateStartEndConstraint()
{
	if (StartConstraintActor != nullptr)
	{
		// StartConstraintActorが設定されていればルートをコンストレイント
		// とりあえずGetActorUpVector()にStartConstraintRadiusだけ離れた位置にコンストレイントさせる形にする
		Positions[0] = InvActorTransform.TransformPosition(StartConstraintActor->GetActorLocation() + StartConstraintActor->GetActorUpVector() * StartConstraintRadius);
	}
	if (EndConstraintActor != nullptr)
	{
		// EndConstraintActorが設定されていれば末端をコンストレイント
		// とりあえずGetActorUpVector()にEndConstraintRadiusだけ離れた位置にコンストレイントさせる形にする
		Positions[Positions.Num() - 1] = InvActorTransform.TransformPosition(EndConstraintActor->GetActorLocation() + EndConstraintActor->GetActorUpVector() * EndConstraintRadius);
	}
	//TODO:長さ制限を与えてConstraintActor自体をコンストレイントさせるのは後で行う:w
}

void ATautRopeSimulatorCPU::UpdateRopeBlockers()
{
	TArray<FOverlapResult> Overlaps;
	FCollisionObjectQueryParams ObjectParams(OverlapQueryObjectTypes);
	if (!ObjectParams.IsValid())
	{
		return;
	}

#if 1
	FCollisionQueryParams Params(SCENE_QUERY_STAT(UpdateRopeBlockers), false);
#else // ComplexコリジョンのTriangleMeshの収集したエッジのデバッグ描画
	FCollisionQueryParams Params(SCENE_QUERY_STAT(UpdateRopeBlockers), true);
#endif

	GetWorld()->OverlapMultiByObjectType(Overlaps, GetActorLocation() + OverlapQueryBox.GetCenter(), GetActorQuat(), ObjectParams, FCollisionShape::MakeBox(OverlapQueryBox.GetExtent()), Params);

	// コリジョンがPrimitiveComponent単位で増減することを想定し、呼ばれるたびにマップを作り直す
	//TODO: PrimitiveComponent内のコリジョンシェイプの増減は想定してない
	OverlapPrimitives.Reset();
	RopeBlockerTriMeshEdgeArray.Reset();

	for (const FOverlapResult& Overlap : Overlaps)
	{
		UPrimitiveComponent* Primitive = Overlap.GetComponent();
		if (Primitive == nullptr || Primitive->GetBodyInstance() == nullptr || Primitive->GetBodyInstance()->GetBodySetup() == nullptr)
		{
			continue;
		}

		OverlapPrimitives.Add(Primitive);

		// ローカル座標に変換
		const FTransform& ActorTM = Primitive->GetComponentTransform() * InvActorTransform;

		//TODO:毎フレームエッジ配列を作る必要はまずない
		TArray<TPair<FVector, FVector>> EdgeArray;

		// TODO:ChaosTriMeshesにも複数のFTriangleMeshImplicitObjectがあるのにすべてまとめてEdgeArrayに入れるのは
		// 無理。
		for (const TSharedPtr<Chaos::FTriangleMeshImplicitObject, ESPMode::ThreadSafe>& TriMesh : Primitive->GetBodyInstance()->GetBodySetup()->ChaosTriMeshes)
		{
			//TODO: 名前空間関数で作りたいが、LargeIdxTypeとSmallIdxTypeの両方で共通の関数を作るのが難しい
			auto CollectEdges = [&](const auto& Triangles)
			{
				// 検索を高速にするためハッシュ構造のあるTSetを使う
				TSet<TPair<int32, int32>> UniqueEdgeSet;

				// 上限サイズをとっておく
				int32 NumTris = Triangles.Num();
				UniqueEdgeSet.Reserve(NumTris * 3);
				EdgeArray.Reserve(NumTris * 3);

				const Chaos::FTriangleMeshImplicitObject::ParticlesType& Vertices = TriMesh->Particles();
		
				// ユニークなエッジ配列を作る処理はFTriangleMesh::GetSegmentMesh()を参考にした
				for (int32 TriIdx = 0; TriIdx < NumTris; TriIdx++)
				{
					for (int32 VertIdx = 0; VertIdx < 3; VertIdx++)
					{
						int32 EdgeIdx0 = Triangles[TriIdx][VertIdx];
						int32 EdgeIdx1 = Triangles[TriIdx][(VertIdx + 1) % 3];
						TPair<int32, int32> Edge;
						if (EdgeIdx0 <= EdgeIdx1)
						{
							Edge = TPair<int32, int32>(EdgeIdx0, EdgeIdx1);
						}
						else
						{
							Edge = TPair<int32, int32>(EdgeIdx1, EdgeIdx0);
						}

						const TPair<int32, int32>* FoundEdge = UniqueEdgeSet.Find(Edge);
						if (FoundEdge == nullptr)
						{
							UniqueEdgeSet.Add(Edge);

							EdgeArray.Add(
								TPair<FVector, FVector>(
									ActorTM.TransformPosition(FVector(Vertices.X(Edge.Key))),
									ActorTM.TransformPosition(FVector(Vertices.X(Edge.Value)))
								)
							);
						}
					}
				}
			};

			const Chaos::FTrimeshIndexBuffer& IdxBuffer = TriMesh->Elements();
			if (IdxBuffer.RequiresLargeIndices())
			{
				CollectEdges(IdxBuffer.GetLargeIndexBuffer());
			}
			else
			{
				CollectEdges(IdxBuffer.GetSmallIndexBuffer());
			}
		}

		//TODO:CollisionStatesで使うために、毎フレーム変化がないこと、要素の順番に変更がないことを前提にしている
		RopeBlockerTriMeshEdgeArray.Append(EdgeArray);
	}
}

namespace NiagaraSandbox::RopeSimulator
{
	// TODO:衝突エッジが変わってなくて衝突位置が変化してるケースに未対応
	enum class ECollisionStateTransition : uint8
	{
		None,
		New,
		Remove,
	};

	struct FCollisionStateTransition
	{
		ECollisionStateTransition Transition = ECollisionStateTransition::None;
		int32 EdgeIdx = INDEX_NONE;
		FVector Point = FVector::ZeroVector;
	};
}

void ATautRopeSimulatorCPU::SolveRopeBlockersCollisionConstraint()
{
	using namespace NiagaraSandbox::RopeSimulator;
#if 1 // ComplexコリジョンのTriangleMeshの収集したエッジのデバッグ描画
	for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
	{
		const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
		const FVector& LineStartWS = GetActorTransform().TransformPosition(Edge.Key);
		const FVector& LineEndWS = GetActorTransform().TransformPosition(Edge.Value);
		DrawDebugLine(GetWorld(), LineStartWS, LineEndWS, FLinearColor::Red.ToFColorSRGB());
	}
#endif

	TArray<FCollisionStateTransition> CollisionStateTransitions;
	CollisionStateTransitions.SetNum(NumParticles);

	for (FCollisionStateTransition& CollisionState : CollisionStateTransitions)
	{
		//TODO:初期化必要？
		CollisionState.Transition = ECollisionStateTransition::None;
		CollisionState.EdgeIdx = INDEX_NONE;
	}

	// TODO:再帰が必要では。ガウスザイデル的反復？

	// 頂点の削除の判定
	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles - 2; ParticleIdx++)
	{
		bool bTraceComplex = false;
		FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraint), bTraceComplex);
		const FVector& Dir = (Positions[ParticleIdx + 2] - Positions[ParticleIdx]).GetSafeNormal();
		// Toleranceだけ線分の長さを双方から縮めるのは、どちらかあるいは両方がエッジと接触
		// していたらヒット判定になるため
		// TODO:もっといい方法ある？
		const FVector& TraceStartWS = GetActorTransform().TransformPosition(Positions[ParticleIdx] + Dir * Tolerance);
		const FVector& TraceEndWS = GetActorTransform().TransformPosition(Positions[ParticleIdx + 2] - Dir * Tolerance);

		//TODO: 衝突するTriMeshが変わった場合への対応は未実装
		bool bIntersectionExist = false;
		for (UPrimitiveComponent* Primitive : OverlapPrimitives)
		{
			FHitResult HitResult;
			bool bHit = Primitive->LineTraceComponent(HitResult, TraceStartWS, TraceEndWS, TraceParams);
			
#if 1 // UPrimitiveComponent::K2_LineTraceComponent()を参考にしている

			DrawDebugLine(GetWorld(), TraceStartWS, bHit ? HitResult.Location : TraceEndWS, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
			if(bHit)
			{
				DrawDebugLine(GetWorld(), HitResult.Location, TraceEndWS, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
			}
#endif

			if (bHit)
			{
				bIntersectionExist = true;
				break;
			}
		}

		if (!bIntersectionExist)
		{
			CollisionStateTransitions[ParticleIdx + 1].Transition = ECollisionStateTransition::Remove;
			// EdgeIdxの初期化はしない。
			// 0-1-2の配列で1がエッジ接触してるときに2を素早く動かして0-2間がコリジョンに
			// 重ならなかったとき、1にRemoveがつく。
			// Cubeの周りをぐるっと囲んだときにそうなりうる。
			// しかし、1-2間で新たなエッジ接触があった場合は1のEdgeIdxと違うインデックスかを判定して
			// 1をNewにしてRemoveを取り消すので

			// 0-1-2の配列で1がエッジ接触してるときに0を素早く動かして0-2間がコリジョンに
			// 重ならなかったとき、1にRemoveがついている。
			// Cubeの周りをぐるっと囲んだときにそうなりうる。
			// しかし、0-1間で新たなエッジ接触があった場合は1のEdgeIdxと違うインデックスかを判定して
			// 0をNewにして1のRemoveを取り消すので

			// Pointの初期化もとりあえずしない。なんの判定にも使っておらず必要ないので
		}
	}

	// 頂点の追加、エッジ移動の判定
	for (int32 ParticleIdx = 0; ParticleIdx < NumSegments; ParticleIdx++)
	{
		// TODO:本当はTriangleでなく扇形で見るべきなんだよな。Triangleだと接触判定で漏らす可能性がある
		// 1フレームだと大きく動かず、扇形をTriangleで近似できる前提のコード
		// ほぼ動いてなければカリング。
		// TODO:すごくゆっくり動かすとコリジョン判定が漏れる可能性があるが、カリングを重視してToleranceSquared
		// によってそれがないように調整する方針
		bool bIntersectionStateChanged = false;
		if ((PrevPositions[ParticleIdx] - Positions[ParticleIdx]).SizeSquared() > ToleranceSquared) // TODO:これは本当に妥当か、バグを生まないか検討
		{
			// 動いている頂点
			const FVector& TriVert0 = PrevPositions[ParticleIdx];
			const FVector& TriVert1 = Positions[ParticleIdx];
			
			// 固定している頂点の方は、動いた頂点の方に少し移動しておく。こうやって、すでに接触している
			// エッジを検出するのを防ぐ
			const FVector& TriVert2 = PrevPositions[ParticleIdx + 1] + (TriVert0 - PrevPositions[ParticleIdx + 1]).GetSafeNormal() * Tolerance;

			for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
			{
				const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
				const FVector& RayStart = Edge.Key;
				const FVector& RayEnd = Edge.Value;
				FVector IntersectPoint;
				FVector IntersectNormal;
				bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
				if (bIntersecting)
				{
					if (EdgeIdxOfPositions[ParticleIdx + 1] != EdgeIdx)
					{
						// 衝突したものが既存の衝突しているエッジと違った場合
						CollisionStateTransitions[ParticleIdx].Transition = ECollisionStateTransition::New;
						// 追加するパーティクルのインデックスはCollisionStateTransitionsのインデックス+1にするというルール
						CollisionStateTransitions[ParticleIdx].EdgeIdx = EdgeIdx;
						CollisionStateTransitions[ParticleIdx].Point = IntersectPoint;
						// 十分細いTriangleだという前提で、二つのエッジに一度に接触するケースは考慮しない
						bIntersectionStateChanged = true;
						// TODO:衝突するエッジが別のエッジに変化した場合に対応してない

						// 0-1-2の配列で1がエッジ接触してるときに0を素早く動かして0-2間がコリジョンに
						// 重ならなかったとき、1にRemoveがついている。
						// Cubeの周りをぐるっと囲んだときにそうなりうる。
						// しかし、0-1間で新たなエッジ接触があった場合は1のRemoveを取り消すべき
						if (CollisionStateTransitions[ParticleIdx + 1].Transition == ECollisionStateTransition::Remove)
						{
							CollisionStateTransitions[ParticleIdx + 1].Transition = ECollisionStateTransition::None;
						}
					}

					// 複数のエッジがまじわる頂点と接触しても最初に検出したエッジのみ使用する
					break;
				}
			}
		}

		// ほぼ動いてなければカリング。
		// TODO:すごくゆっくり動かすとコリジョン判定が漏れる可能性があるが、カリングを重視してToleranceSquared
		// によってそれがないように調整する方針
		if (!bIntersectionStateChanged // 一方のTriangleで接触変更を検知したらもう一方は判定しない
			&& (PrevPositions[ParticleIdx + 1] - Positions[ParticleIdx + 1]).SizeSquared() > ToleranceSquared)
		{
			// 動いている頂点
			const FVector& TriVert1 = Positions[ParticleIdx + 1];
			const FVector& TriVert2 = PrevPositions[ParticleIdx + 1];
			
			// 固定している頂点の方は、動いた頂点の方に少し移動しておく。こうやって、すでに接触している
			// エッジを検出するのを防ぐ
			const FVector& TriVert0 = PrevPositions[ParticleIdx] + (TriVert2 - PrevPositions[ParticleIdx]).GetSafeNormal() * Tolerance;

			// TODO: 上のifブロックと処理が冗長
			for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
			{
				const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
				const FVector& RayStart = Edge.Key;
				const FVector& RayEnd = Edge.Value;
				FVector IntersectPoint;
				FVector IntersectNormal;
				bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
				if (bIntersecting)
				{
					if (EdgeIdxOfPositions[ParticleIdx] != EdgeIdx)
					{
						// 衝突したものが既存の衝突しているエッジと違った場合
						CollisionStateTransitions[ParticleIdx].Transition = ECollisionStateTransition::New;
						// TODO:既にCollisionStateTransitions[ParticleIdx].Transition==ECollisionStateTransition::Removeだったケースに対応してない
						// 追加するパーティクルのインデックスはCollisionStateTransitionsのインデックス+1にするというルール
						CollisionStateTransitions[ParticleIdx].EdgeIdx = EdgeIdx;
						CollisionStateTransitions[ParticleIdx].Point = IntersectPoint;
						// 十分細いTriangleだという前提で、二つのエッジに一度に接触するケースは考慮しない
						bIntersectionStateChanged = true;
						// TODO:衝突するエッジが別のエッジに変化した場合に対応してない
					}

					// 複数のエッジがまじわる頂点と接触しても最初に検出したエッジのみ使用する
					break;
				}
			}
		}
	}

	int32 IncreasedCount = 0; // 衝突が減ったときは負になる
	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ParticleIdx++)
	{
		const FCollisionStateTransition& CollisionStateTransition = CollisionStateTransitions[ParticleIdx];
		switch (CollisionStateTransition.Transition)
		{
			case ECollisionStateTransition::None:
				// 何もしない
				break;
			case ECollisionStateTransition::New:
				PrevPositions.Insert(CollisionStateTransition.Point, ParticleIdx + 1 + IncreasedCount);
				Positions.Insert(CollisionStateTransition.Point, ParticleIdx + 1 + IncreasedCount);
				EdgeIdxOfPositions.Insert(CollisionStateTransition.EdgeIdx, ParticleIdx + 1 + IncreasedCount);
				IncreasedCount++;
				break;
			case ECollisionStateTransition::Remove:
				PrevPositions.RemoveAt(ParticleIdx + IncreasedCount);
				Positions.RemoveAt(ParticleIdx + IncreasedCount);
				EdgeIdxOfPositions.RemoveAt(ParticleIdx + IncreasedCount);
				IncreasedCount--;
				break;
			default:
				check(false);
				break;
		}
	}

	NumParticles += IncreasedCount;
	NumSegments = NumParticles - 1;
}

ATautRopeSimulatorCPU::ATautRopeSimulatorCPU()
{
	PrimaryActorTick.bCanEverTick = true;
	// EndConstraintActorの物理シミュレーションが終わった後にその位置に応じてシミュレーションしたいので
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	NiagaraComponent = CreateDefaultSubobject<UNiagaraComponent>(TEXT("NiagaraComponent0"));

	RootComponent = NiagaraComponent;

#if WITH_EDITORONLY_DATA
	SpriteComponent = CreateEditorOnlyDefaultSubobject<UBillboardComponent>(TEXT("Sprite"));
	ArrowComponent = CreateEditorOnlyDefaultSubobject<UArrowComponent>(TEXT("ArrowComponent0"));

	if (!IsRunningCommandlet())
	{
		// Structure to hold one-time initialization
		struct FConstructorStatics
		{
			ConstructorHelpers::FObjectFinderOptional<UTexture2D> SpriteTextureObject;
			FName ID_Effects;
			FText NAME_Effects;
			FConstructorStatics()
				: SpriteTextureObject(TEXT("/Niagara/Icons/S_ParticleSystem"))
				, ID_Effects(TEXT("Effects"))
				, NAME_Effects(NSLOCTEXT("SpriteCategory", "Effects", "Effects"))
			{
			}
		};
		static FConstructorStatics ConstructorStatics;

		if (SpriteComponent)
		{
			SpriteComponent->Sprite = ConstructorStatics.SpriteTextureObject.Get();
			SpriteComponent->SetRelativeScale3D_Direct(FVector(0.5f, 0.5f, 0.5f));
			SpriteComponent->bHiddenInGame = true;
			SpriteComponent->bIsScreenSizeScaled = true;
			SpriteComponent->SpriteInfo.Category = ConstructorStatics.ID_Effects;
			SpriteComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Effects;
			SpriteComponent->SetupAttachment(NiagaraComponent);
			SpriteComponent->bReceivesDecals = false;
		}

		if (ArrowComponent)
		{
			ArrowComponent->ArrowColor = FColor(0, 255, 128);

			ArrowComponent->ArrowSize = 1.5f;
			ArrowComponent->bTreatAsASprite = true;
			ArrowComponent->bIsScreenSizeScaled = true;
			ArrowComponent->SpriteInfo.Category = ConstructorStatics.ID_Effects;
			ArrowComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Effects;
			ArrowComponent->SetupAttachment(NiagaraComponent);
			ArrowComponent->SetUsingAbsoluteScale(true);
		}
	}
#endif // WITH_EDITORONLY_DATA
}

void ATautRopeSimulatorCPU::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	// Set Notification Delegate
	if (NiagaraComponent)
	{
		NiagaraComponent->OnSystemFinished.AddUniqueDynamic(this, &ATautRopeSimulatorCPU::OnNiagaraSystemFinished);
	}
}

void ATautRopeSimulatorCPU::SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish)
{
	bDestroyOnSystemFinish = bShouldDestroyOnSystemFinish ? 1 : 0;  
};

void ATautRopeSimulatorCPU::OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent)
{
	if (bDestroyOnSystemFinish)
	{
		SetLifeSpan(0.0001f);
	}
}

#if WITH_EDITOR
bool ATautRopeSimulatorCPU::GetReferencedContentObjects(TArray<UObject*>& Objects) const
{
	Super::GetReferencedContentObjects(Objects);

	if (UNiagaraSystem* System = NiagaraComponent->GetAsset())
	{
		Objects.Add(System);
	}

	return true;
}

void ATautRopeSimulatorCPU::ResetInLevel()
{
	if (NiagaraComponent)
	{
		NiagaraComponent->Activate(true);
		NiagaraComponent->ReregisterComponent();
	}
}
#endif // WITH_EDITOR


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

void ATautRopeSimulatorCPU::BeginPlay()
{
	Super::BeginPlay();

	// 何度も使うのでキャッシュしておく
	ToleranceSquared = Tolerance * Tolerance; // TODO:リアルタイムには更新しない

	// まずは一つの線分から
	int32 NumParticles = 2;
	int32 NumSegments = NumParticles - 1;

	Positions.SetNum(NumParticles);
	PrevPositions.SetNum(NumParticles);
	ParentPositions.SetNum(NumSegments);
	ChildPositions.SetNum(NumSegments);
	EdgeIdxOfPositions.SetNum(NumParticles);
	MovedFlagOfPositions.SetNum(NumParticles);

	UpdateStartEndConstraint();

	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ParticleIdx++)
	{
		PrevPositions[ParticleIdx] = Positions[ParticleIdx];
		EdgeIdxOfPositions[ParticleIdx] = INDEX_NONE;
		MovedFlagOfPositions[ParticleIdx] = false;
	}

	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles - 1; ParticleIdx++)
	{
		ParentPositions[ParticleIdx] = Positions[ParticleIdx];
		ChildPositions[ParticleIdx] = Positions[ParticleIdx + 1];
	}
}

void ATautRopeSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		UpdateStartEndConstraint();
		UpdateRopeBlockers();
		//SolveRopeBlockersCollisionConstraintOld();
		SolveRopeBlockersCollisionConstraint();
	}

	check(Positions.Num() >= 2);
	check(Positions.Num() == PrevPositions.Num());
	// 直接サイズとインデックス指定でコピーする方法がないので一旦別変数にコピーしてMove。
	int32 NumSegments = Positions.Num() - 1;
	TArray<FVector> TmpParentPositions(Positions.GetData(), NumSegments);
	TArray<FVector> TmpChildPositions(&Positions.GetData()[1], NumSegments);
	ParentPositions = MoveTemp(TmpParentPositions);
	ChildPositions = MoveTemp(TmpChildPositions);
	NiagaraComponent->SetNiagaraVariableInt("NumSegments", NumSegments);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ParentPositions"), ParentPositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ChildPositions"), ChildPositions);
}

void ATautRopeSimulatorCPU::UpdateStartEndConstraint()
{
	if (StartConstraintComponent != nullptr)
	{
		Positions[0] = InvActorTransform.TransformPosition(StartConstraintComponent->GetSocketLocation(StartConstraintSocket));
	}
	else if (StartConstraintActor != nullptr)
	{
		Positions[0] = InvActorTransform.TransformPosition(StartConstraintActor->GetRootComponent()->GetSocketLocation(StartConstraintSocket));
	}

	if (EndConstraintComponent != nullptr)
	{
		Positions[Positions.Num() - 1] = InvActorTransform.TransformPosition(EndConstraintComponent->GetSocketLocation(EndConstraintSocket));
	}
	else if (EndConstraintActor != nullptr)
	{
		Positions[Positions.Num() - 1] = InvActorTransform.TransformPosition(EndConstraintActor->GetRootComponent()->GetSocketLocation(EndConstraintSocket));
	}
	//TODO:長さ制限を与えてConstraintActor自体をコンストレイントさせるのは後で行う
}

void ATautRopeSimulatorCPU::UpdateRopeBlockers()
{
	TArray<FOverlapResult> Overlaps;
	FCollisionObjectQueryParams ObjectParams(OverlapQueryObjectTypes);
	if (!ObjectParams.IsValid())
	{
		return;
	}

	// OverlapクエリはSimpleコリジョンへのクエリで十分と判断
	FCollisionQueryParams Params(SCENE_QUERY_STAT(UpdateRopeBlockers), false);

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
	//TODO: FMath::PointDistToSegmentSquared()が結果をfloatにキャストして戻り値にしてるのでしょうがなく
	double PointDistToSegmentSquared(const FVector &Point, const FVector &StartPoint, const FVector &EndPoint)
	{
		const FVector& ClosestPoint = FMath::ClosestPointOnSegment(Point, StartPoint, EndPoint);
		return (Point - ClosestPoint).SizeSquared();
	}
}

void ATautRopeSimulatorCPU::SolveRopeBlockersCollisionConstraintOld()
{
	using namespace NiagaraSandbox::RopeSimulator;

	if (bDrawCollisionEdge)
	{
		for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
		{
			const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
			const FVector& LineStartWS = GetActorTransform().TransformPosition(Edge.Key);
			const FVector& LineEndWS = GetActorTransform().TransformPosition(Edge.Value);
			DrawDebugLine(GetWorld(), LineStartWS, LineEndWS, FLinearColor::Red.ToFColorSRGB());
		}
	}

	// MovementPhaseとCollisionPhaseの全頂点のイテレーション。収束するまでループする。
	bool bExistMovedParticle = false;
	if (Positions[0] != PrevPositions[0]
		|| Positions[Positions.Num() - 1] != PrevPositions[Positions.Num() - 1]) // TODO:Toleranceを入れないでみる
	{
		bExistMovedParticle = true;
	}

	for (int32 IterCount = 0; IterCount < MaxIteration && bExistMovedParticle; IterCount++)
	{
		bool bExistAddedParticle = false;

		// TODO: 逆方向ループはあとで必要か検討する
		for (int32 ParticleIdx = 0; ParticleIdx < Positions.Num(); ParticleIdx = ((bExistAddedParticle && ParticleIdx == 0) ? 0 : ParticleIdx + 1)) // 始点のときのみ頂点追加があったらもう一度始点で行う
		{
			//
			// MovementPhase
			//
			bool bNeedCollisionPhase = false;
			MovedFlagOfPositions[ParticleIdx] = false;
			{
				if (ParticleIdx == 0)
				{
					if (Positions[0] != PrevPositions[0]) // TODO:Toleranceを入れないでみる
					{
						bNeedCollisionPhase = true;
					}

					// 始点と終点は1フレームに一度しか動かさないので即Movedフラグを下げる
					MovedFlagOfPositions[ParticleIdx] = false;
				}
				else if (ParticleIdx == (Positions.Num() - 1))
				{
					if (Positions[Positions.Num() - 1] != PrevPositions[Positions.Num() - 1]) // TODO:Toleranceを入れないでみる
					{
						bNeedCollisionPhase = true;
					}

					// 始点と終点は1フレームに一度しか動かさないので即Movedフラグを下げる
					MovedFlagOfPositions[ParticleIdx] = false;
				}
				else // 始点と終点以外は最短コンストレイント
				{
					check(EdgeIdxOfPositions[ParticleIdx] != INDEX_NONE);
					const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdxOfPositions[ParticleIdx]];
					const FVector& EdgeStart = Edge.Key;
					const FVector& EdgeEnd = Edge.Value;

					const FVector& StartPoint = Positions[ParticleIdx - 1];
					const FVector& EndPoint = Positions[ParticleIdx + 1];

					// 動いてるのがStartPointであろうとEndPointであろうとStartPointPlane側を回転させて
					// 最短交点を計算する

					// エッジの直線に垂線を下ろした点 // TODO:エッジ移動を考えないので線分でなく直線で足を求めている
					const FVector& StartPointDropFoot = FMath::ClosestPointOnInfiniteLine(EdgeStart, EdgeEnd, StartPoint);
					const FVector& EndPointDropFoot = FMath::ClosestPointOnInfiniteLine(EdgeStart, EdgeEnd, EndPoint);

					double StartPointPerpLen = (StartPoint - StartPointDropFoot).Size();
					double EndPointPerpLen = (EndPoint - EndPointDropFoot).Size();
					
					// 最短になる交点は垂線の長さの比による線形補間で決まる
					double Alpha = StartPointPerpLen / (StartPointPerpLen + EndPointPerpLen);
					Positions[ParticleIdx] = FMath::Lerp(StartPointDropFoot, EndPointDropFoot, Alpha);
					// TODO:エッジ移動を考えないので線分外に出たらログを出しておく
					if (Alpha <= 0.0 || Alpha >= 1.0)
					{
						UE_LOG(LogTemp, Log,  TEXT("Alpha = %lf. Shortest constraint generates overgoing edge."), Alpha);
					}

					// TODO: エッジ移動は考えず、頂点追加も現状始点終点でないセグメントでは起こさない前提で、コリジョンフェイズに入れない
					bNeedCollisionPhase = false;
					// 収束のため閾値つきで動いたかどうか判定
					MovedFlagOfPositions[ParticleIdx] = ((PrevPositions[ParticleIdx] - Positions[ParticleIdx]).SizeSquared() > ToleranceSquared);
				}
			}

			//
			// CollisionPhase
			//
			// 追加頂点があると始点終点はその延長までしか動かさず再度MovementPhaseをやるので延長位置を保存する
			bExistAddedParticle = false;
			FVector MovePosition = Positions[ParticleIdx];
			if (bNeedCollisionPhase)
			{
				// TODO:実装
				// 単に動いた時の前後セグメントでの頂点追加
				// TODO:エッジ移動とエッジに沿った削除はあとで実装する
				// TODO:現状、頂点追加は始点と終点のセグメント以外では考えない
				if (ParticleIdx == 0)
				{
					// 動いている頂点
					const FVector& TriVert0 = PrevPositions[ParticleIdx];
					const FVector& TriVert1 = Positions[ParticleIdx];
					const FVector& TriVert2 = PrevPositions[ParticleIdx + 1];

					double NearestEdgeDistanceSq = DBL_MAX;
					FVector NearestIntersectPoint = FVector::ZeroVector;
					int32 NearestEdgeIdx = INDEX_NONE;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						// 異なるエッジでないものは採用しない
						if (EdgeIdxOfPositions[ParticleIdx] == EdgeIdx || EdgeIdxOfPositions[ParticleIdx + 1] == EdgeIdx)
						{
							continue;
						}

						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& RayStart = Edge.Key;
						const FVector& RayEnd = Edge.Value;
						FVector IntersectPoint;
						FVector IntersectNormal;
						bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
						if (bIntersecting)
						{
							// 元の線分と最も近いエッジを採用
							double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(IntersectPoint, TriVert0, TriVert2);
							if (EdgeDistanceSq < NearestEdgeDistanceSq)
							{
								// 等距離なら最も若いインデックスを採用する
								NearestEdgeDistanceSq = EdgeDistanceSq;
								NearestIntersectPoint = IntersectPoint;
								NearestEdgeIdx = EdgeIdx;
							}
						}
					}

					// 頂点追加
					if (NearestEdgeIdx != INDEX_NONE)
					{
						// TODO:GDC動画のように延長線上を新たな終点におくと、同一平面に複数エッジがある
						// シェイプだと、同一平面上のエッジを拾い損ねることがFMath::SegmentTriangleIntersection()では簡単に起きる。
						// Triangleの辺にエッジがある場合に交差を検出失敗する。
						// よってPrevPositionsの更新は頂点の追加が終わるまでしない
#if 0
						// TODO:TriVert0と1の間の点ではなく長さと向きを維持した延長線上の点にしている
						PrevPositions[ParticleIdx] = TriVert2 + (NearestIntersectPoint - TriVert2).GetSafeNormal() * (TriVert0 - TriVert2).Size();
#endif

						PrevPositions.Insert(NearestIntersectPoint, ParticleIdx + 1);
						Positions.Insert(NearestIntersectPoint, ParticleIdx + 1);
						EdgeIdxOfPositions.Insert(NearestEdgeIdx, ParticleIdx + 1);
						MovedFlagOfPositions.Insert(true, ParticleIdx + 1); // ここでtrueにしてもパーティクルループの自分の番のMovementPhaseで別途判定される

						// MoveHalfwayPositionからPositions[ParticleIdx]の間で動かしてさらに他のエッジ接触がないかチェックする
						bExistAddedParticle = true;
					}

					// エッジからはがす削除
					// TODO:終点と処理が冗長
					if (Positions.Num() >= 3)
					{
						// 0-2の線分の削除だけで判定していないのは、なにかに1をひっかけている状態で
						// 0-2を大きく動かすと0-2線分上にコリジョンがいない状態になり削除対象になってしまうから
						// TODO:本当は3つのライントレースでなくTriangleとコリジョンのOverlapをとりたかったがAPIがなかった
						// TODO:もっといいやり方あるかも

						bool bTraceComplex = false;
						FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraintOld), bTraceComplex);
						// Toleranceだけ0と2を双方から縮めるのは、どちらかあるいは両方がエッジと接触
						// していたらそのエッジとヒット判定になり、1の削除判定をしたいのにできないため
						// TODO:もっといい方法ある？
						// Toleranceだけ1を0-2の線分側に近づけるのは、1がエッジと接触してると接触判定し続けるため
						// TODO:もっといい方法ある？
						const FVector& SmallTriVert0 = Positions[2] + (Positions[0] - Positions[2]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert1 = Positions[1] + ((Positions[2] + Positions[0]) * 0.5 - Positions[1]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert2 = Positions[0] + (Positions[2] - Positions[0]).GetSafeNormal() * Tolerance;

						const FVector& SmallTriVertWS0 = GetActorTransform().TransformPosition(SmallTriVert0);
						const FVector& SmallTriVertWS1 = GetActorTransform().TransformPosition(SmallTriVert1);
						const FVector& SmallTriVertWS2 = GetActorTransform().TransformPosition(SmallTriVert2);

						bool bIntersectionExist = false;
						for (UPrimitiveComponent* Primitive : OverlapPrimitives)
						{
							FHitResult HitResult;
							bool bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS0, SmallTriVertWS1, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS0, bHit ? HitResult.Location : SmallTriVertWS1, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS1, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS1, SmallTriVertWS2, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS1, bHit ? HitResult.Location : SmallTriVertWS2, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS2, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS2, SmallTriVertWS0, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS2, bHit ? HitResult.Location : SmallTriVertWS0, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS0, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}
						}

						if (!bIntersectionExist)
						{
							PrevPositions.RemoveAt(1);
							Positions.RemoveAt(1);
							EdgeIdxOfPositions.RemoveAt(1);
							MovedFlagOfPositions.RemoveAt(1);

							// 追加したものが即削除された場合ももう追加判定はとめて次のパーティクルに進む。
							// 追加と即削除を繰り返すとParticleIdx=0のままでループが進まないので。
							bExistAddedParticle = false;
						}
					}

					if (!bExistAddedParticle)
					{
						PrevPositions[0] = Positions[0];
					}
				}
				else if (ParticleIdx == (Positions.Num() - 1))
				{
					// 動いている頂点
					const FVector& TriVert1 = Positions[ParticleIdx];
					const FVector& TriVert2 = PrevPositions[ParticleIdx];
					
					// 固定している頂点は始点の判定と違ってPrevPositonsでなくPositionsなのに注意。
					// こうしないと1フレームでParticlesIdxの頂点が大きく動いたとき交差検出が漏れるケースがある
					// https://www.gdcvault.com/play/1027351/Rope-Simulation-in-Uncharted-4
					// の32分ごろの例。
					const FVector& TriVert0 = Positions[ParticleIdx - 1];

					// TODO: 上のifブロックと処理が冗長
					double NearestEdgeDistanceSq = DBL_MAX;
					FVector NearestIntersectPoint = FVector::ZeroVector;
					int32 NearestEdgeIdx = INDEX_NONE;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						// 異なるエッジでないものは採用しない
						if (EdgeIdxOfPositions[ParticleIdx - 1] == EdgeIdx || EdgeIdxOfPositions[ParticleIdx] == EdgeIdx)
						{
							continue;
						}

						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& RayStart = Edge.Key;
						const FVector& RayEnd = Edge.Value;
						FVector IntersectPoint;
						FVector IntersectNormal;
						bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
						if (bIntersecting)
						{
							// 元の線分と最も近いエッジを採用
							double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(IntersectPoint, TriVert0, TriVert2);
							if (EdgeDistanceSq < NearestEdgeDistanceSq)
							{
								// 等距離なら最も若いインデックスを採用する
								NearestEdgeDistanceSq = EdgeDistanceSq;
								NearestIntersectPoint = IntersectPoint;
								NearestEdgeIdx = EdgeIdx;
							}
						}
					}

					if (NearestEdgeIdx != INDEX_NONE)
					{
						// TODO:GDC動画のように延長線上を新たな終点におくと、同一平面に複数エッジがある
						// シェイプだと、同一平面上のエッジを拾い損ねることがFMath::SegmentTriangleIntersection()では簡単に起きる。
						// Triangleの辺にエッジがある場合に交差を検出失敗する。
						// よってPrevPositionsの更新は頂点の追加が終わるまでしない
#if 0
						// TODO:TriVert0と1の間の点ではなく長さと向きを維持した延長線上の点にしている
						PrevPositions[ParticleIdx] = TriVert0 + (NearestIntersectPoint - TriVert0).GetSafeNormal() * (TriVert2 - TriVert0).Size();
#endif

						PrevPositions.Insert(NearestIntersectPoint, ParticleIdx);
						Positions.Insert(NearestIntersectPoint, ParticleIdx);
						EdgeIdxOfPositions.Insert(NearestEdgeIdx, ParticleIdx);
						MovedFlagOfPositions.Insert(true, ParticleIdx); // ここでtrueにして次イテレーションで最短コンストレイントを行う // TODO:逆順ループを作れば次イテレーションにしなくてもよくなり収束も早くなるかも

						// MoveHalfwayPositionからPositions[ParticleIdx]の間で動かしてさらに他のエッジ接触がないかチェックする
						bExistAddedParticle = true;
					}

					// エッジからはがす削除
					// TODO:終点と処理が冗長
					if (Positions.Num() >= 3)
					{
						// 0-2の線分の削除だけで判定していないのは、なにかに1をひっかけている状態で
						// 0-2を大きく動かすと0-2線分上にコリジョンがいない状態になり削除対象になってしまうから
						// TODO:本当は3つのライントレースでなくTriangleとコリジョンのOverlapをとりたかったがAPIがなかった
						// TODO:もっといいやり方あるかも

						bool bTraceComplex = false;
						FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraintOld), bTraceComplex);
						// Toleranceだけ0と2を双方から縮めるのは、どちらかあるいは両方がエッジと接触
						// していたらそのエッジとヒット判定になり、1の削除判定をしたいのにできないため
						// TODO:もっといい方法ある？
						// Toleranceだけ1を0-2の線分側に近づけるのは、1がエッジと接触してると接触判定し続けるため
						// TODO:もっといい方法ある？
						const FVector& SmallTriVert0 = Positions[Positions.Num() - 1] + (Positions[Positions.Num() - 3] - Positions[Positions.Num() - 1]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert1 = Positions[Positions.Num() - 1 - 1] + ((Positions[Positions.Num() - 1] + Positions[Positions.Num() - 3]) * 0.5 - Positions[Positions.Num() - 2]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert2 = Positions[Positions.Num() - 3] + (Positions[Positions.Num() - 1] - Positions[Positions.Num() - 3]).GetSafeNormal() * Tolerance;

						const FVector& SmallTriVertWS0 = GetActorTransform().TransformPosition(SmallTriVert0);
						const FVector& SmallTriVertWS1 = GetActorTransform().TransformPosition(SmallTriVert1);
						const FVector& SmallTriVertWS2 = GetActorTransform().TransformPosition(SmallTriVert2);

						bool bIntersectionExist = false;
						for (UPrimitiveComponent* Primitive : OverlapPrimitives)
						{
							FHitResult HitResult;
							bool bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS0, SmallTriVertWS1, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS0, bHit ? HitResult.Location : SmallTriVertWS1, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS1, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS1, SmallTriVertWS2, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS1, bHit ? HitResult.Location : SmallTriVertWS2, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS2, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS2, SmallTriVertWS0, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS2, bHit ? HitResult.Location : SmallTriVertWS0, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS0, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}
						}

						if (!bIntersectionExist)
						{
							int32 LastIdx = Positions.Num() - 2;
							PrevPositions.RemoveAt(LastIdx);
							Positions.RemoveAt(LastIdx);
							EdgeIdxOfPositions.RemoveAt(LastIdx);
							MovedFlagOfPositions.RemoveAt(LastIdx);

							bExistAddedParticle = false;
						}
					}

					if (!bExistAddedParticle)
					{
						PrevPositions[Positions.Num() - 1] = Positions[Positions.Num() - 1];
					}
				}
				else
				{
					check(false);
				}
			}
			else
			{
				check(!bExistAddedParticle);
				PrevPositions[ParticleIdx] = Positions[ParticleIdx];
			}
		}

		bExistMovedParticle = false;
		for (int32 ParticleIdx = 0; ParticleIdx < Positions.Num() - 1; ParticleIdx++)
		{
			if (MovedFlagOfPositions[ParticleIdx])
			{
				bExistMovedParticle = true;
				break;
			}
		}
	}
}

void ATautRopeSimulatorCPU::SolveRopeBlockersCollisionConstraint()
{
	using namespace NiagaraSandbox::RopeSimulator;

	if (bDrawCollisionEdge)
	{
		for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
		{
			const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
			const FVector& LineStartWS = GetActorTransform().TransformPosition(Edge.Key);
			const FVector& LineEndWS = GetActorTransform().TransformPosition(Edge.Value);
			DrawDebugLine(GetWorld(), LineStartWS, LineEndWS, FLinearColor::Red.ToFColorSRGB());
		}
	}

	// MovementPhaseとCollisionPhaseの全頂点のイテレーション。収束するまでループする。
	bool bExistMovedParticle = false;
	if (Positions[0] != PrevPositions[0]
		|| Positions[Positions.Num() - 1] != PrevPositions[Positions.Num() - 1]) // TODO:Toleranceを入れないでみる
	{
		bExistMovedParticle = true;
	}

	for (int32 IterCount = 0; IterCount < MaxIteration && bExistMovedParticle; IterCount++)
	{
		bool bExistAddedParticle = false;

		// TODO: 逆方向ループはあとで必要か検討する
		for (int32 ParticleIdx = 0; ParticleIdx < Positions.Num(); ParticleIdx = ((bExistAddedParticle && ParticleIdx == 0) ? 0 : ParticleIdx + 1)) // 始点のときのみ頂点追加があったらもう一度始点で行う
		{
			//
			// MovementPhase
			//
			bool bNeedCollisionPhase = false;
			MovedFlagOfPositions[ParticleIdx] = false;
			{
				if (ParticleIdx == 0)
				{
					bNeedCollisionPhase = (Positions[0] != PrevPositions[0]); // TODO:Toleranceを入れないでみる

					// 始点と終点は1フレームに一度しか動かさないので即Movedフラグを下げる
					MovedFlagOfPositions[ParticleIdx] = false;
				}
				else if (ParticleIdx == (Positions.Num() - 1))
				{
					bNeedCollisionPhase = (Positions[Positions.Num() - 1] != PrevPositions[Positions.Num() - 1]); // TODO:Toleranceを入れないでみる

					// 始点と終点は1フレームに一度しか動かさないので即Movedフラグを下げる
					MovedFlagOfPositions[ParticleIdx] = false;
				}
				else // 始点と終点以外は最短コンストレイント
				{
					check(EdgeIdxOfPositions[ParticleIdx] != INDEX_NONE);
					const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdxOfPositions[ParticleIdx]];
					const FVector& EdgeStart = Edge.Key;
					const FVector& EdgeEnd = Edge.Value;

					const FVector& StartPoint = Positions[ParticleIdx - 1];
					const FVector& EndPoint = Positions[ParticleIdx + 1];

					// 動いてるのがStartPointであろうとEndPointであろうとStartPointPlane側を回転させて
					// 最短交点を計算する

					// エッジの直線に垂線を下ろした点をエッジを直線にして計算
					const FVector& StartPointDropFoot = FMath::ClosestPointOnInfiniteLine(EdgeStart, EdgeEnd, StartPoint);
					const FVector& EndPointDropFoot = FMath::ClosestPointOnInfiniteLine(EdgeStart, EdgeEnd, EndPoint);

					double StartPointPerpLen = (StartPoint - StartPointDropFoot).Size();
					double EndPointPerpLen = (EndPoint - EndPointDropFoot).Size();
					
					// 最短になる交点は垂線の長さの比による線形補間で決まる
					double Alpha = StartPointPerpLen / (StartPointPerpLen + EndPointPerpLen);
					const FVector& ShortestPointOnInfiniteLine = FMath::Lerp(StartPointDropFoot, EndPointDropFoot, Alpha);
					double EdgeLenSqr = FVector::DotProduct(EdgeEnd - EdgeStart, EdgeEnd - EdgeStart);

					// エッジは線分なのでクランプする
					bool bClamped = false;
					if (FVector::DotProduct(ShortestPointOnInfiniteLine - EdgeStart, EdgeEnd - EdgeStart) > EdgeLenSqr) // TODO:Toleranceを入れないでみる
					{
						Positions[ParticleIdx] = EdgeEnd;
						bClamped = true;
					}
					else if (FVector::DotProduct(ShortestPointOnInfiniteLine - EdgeEnd, EdgeStart - EdgeEnd) > EdgeLenSqr) // TODO:Toleranceを入れないでみる
					{
						Positions[ParticleIdx] = EdgeStart;
						bClamped = true;
					}
					else
					{
						Positions[ParticleIdx] = ShortestPointOnInfiniteLine;
						bClamped = false;
					}

					// エッジの端にクランプされたとき、動いていたらCollisionPhaseへ。属すべきエッジの変化が起きうるので、エッジ移動、頂点削除、追加などの判定を行う。
					bNeedCollisionPhase = bClamped && (Positions[ParticleIdx] != PrevPositions[ParticleIdx]); // TODO:Toleranceを入れないでみる

					// 収束のため閾値つきで動いたかどうか判定
					MovedFlagOfPositions[ParticleIdx] = ((PrevPositions[ParticleIdx] - Positions[ParticleIdx]).SizeSquared() > ToleranceSquared);
				}
			}

			//
			// CollisionPhase
			//
			// 追加頂点があると始点終点はその延長までしか動かさず再度MovementPhaseをやるので延長位置を保存する
			bExistAddedParticle = false;
			FVector MovePosition = Positions[ParticleIdx];
			if (bNeedCollisionPhase)
			{
				// 単に動いた時の前後セグメントでの頂点追加
				// TODO:エッジ移動とエッジに沿った削除はあとで実装する
				// TODO:現状、頂点追加は始点と終点のセグメント以外では考えない
				if (ParticleIdx == 0)
				{
					// 動いている頂点
					const FVector& TriVert0 = PrevPositions[ParticleIdx];
					const FVector& TriVert1 = Positions[ParticleIdx];
					const FVector& TriVert2 = PrevPositions[ParticleIdx + 1];

					double NearestEdgeDistanceSq = DBL_MAX;
					FVector NearestIntersectPoint = FVector::ZeroVector;
					int32 NearestEdgeIdx = INDEX_NONE;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						// 異なるエッジでないものは採用しない
						if (EdgeIdxOfPositions[ParticleIdx] == EdgeIdx || EdgeIdxOfPositions[ParticleIdx + 1] == EdgeIdx)
						{
							continue;
						}

						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& RayStart = Edge.Key;
						const FVector& RayEnd = Edge.Value;
						FVector IntersectPoint;
						FVector IntersectNormal;
						bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
						if (bIntersecting)
						{
							// 元の線分と最も近いエッジを採用
							double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(IntersectPoint, TriVert0, TriVert2);
							if (EdgeDistanceSq < NearestEdgeDistanceSq)
							{
								// 等距離なら最も若いインデックスを採用する
								NearestEdgeDistanceSq = EdgeDistanceSq;
								NearestIntersectPoint = IntersectPoint;
								NearestEdgeIdx = EdgeIdx;
							}
						}
					}

					// 頂点追加
					if (NearestEdgeIdx != INDEX_NONE)
					{
						// TODO:GDC動画のように延長線上を新たな終点におくと、同一平面に複数エッジがある
						// シェイプだと、同一平面上のエッジを拾い損ねることがFMath::SegmentTriangleIntersection()では簡単に起きる。
						// Triangleの辺にエッジがある場合に交差を検出失敗する。
						// よってPrevPositionsの更新は頂点の追加が終わるまでしない
						PrevPositions.Insert(NearestIntersectPoint, ParticleIdx + 1);
						Positions.Insert(NearestIntersectPoint, ParticleIdx + 1);
						EdgeIdxOfPositions.Insert(NearestEdgeIdx, ParticleIdx + 1);
						MovedFlagOfPositions.Insert(true, ParticleIdx + 1); // ここでtrueにしてもパーティクルループの自分の番のMovementPhaseで別途判定される

						// MoveHalfwayPositionからPositions[ParticleIdx]の間で動かしてさらに他のエッジ接触がないかチェックする
						bExistAddedParticle = true;
					}

					// エッジからはがす削除
					// TODO:終点と処理が冗長
					if (Positions.Num() >= 3)
					{
						// 0-2の線分の削除だけで判定していないのは、なにかに1をひっかけている状態で
						// 0-2を大きく動かすと0-2線分上にコリジョンがいない状態になり削除対象になってしまうから
						// TODO:本当は3つのライントレースでなくTriangleとコリジョンのOverlapをとりたかったがAPIがなかった
						// TODO:もっといいやり方あるかも

						bool bTraceComplex = false;
						FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraint), bTraceComplex);
						// Toleranceだけ0と2を双方から縮めるのは、どちらかあるいは両方がエッジと接触
						// していたらそのエッジとヒット判定になり、1の削除判定をしたいのにできないため
						// TODO:もっといい方法ある？
						// Toleranceだけ1を0-2の線分側に近づけるのは、1がエッジと接触してると接触判定し続けるため
						// TODO:もっといい方法ある？
						const FVector& SmallTriVert0 = Positions[2] + (Positions[0] - Positions[2]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert1 = Positions[1] + ((Positions[2] + Positions[0]) * 0.5 - Positions[1]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert2 = Positions[0] + (Positions[2] - Positions[0]).GetSafeNormal() * Tolerance;

						const FVector& SmallTriVertWS0 = GetActorTransform().TransformPosition(SmallTriVert0);
						const FVector& SmallTriVertWS1 = GetActorTransform().TransformPosition(SmallTriVert1);
						const FVector& SmallTriVertWS2 = GetActorTransform().TransformPosition(SmallTriVert2);

						bool bIntersectionExist = false;
						for (UPrimitiveComponent* Primitive : OverlapPrimitives)
						{
							FHitResult HitResult;
							bool bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS0, SmallTriVertWS1, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS0, bHit ? HitResult.Location : SmallTriVertWS1, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS1, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS1, SmallTriVertWS2, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS1, bHit ? HitResult.Location : SmallTriVertWS2, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS2, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS2, SmallTriVertWS0, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS2, bHit ? HitResult.Location : SmallTriVertWS0, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS0, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}
						}

						if (!bIntersectionExist)
						{
							PrevPositions.RemoveAt(1);
							Positions.RemoveAt(1);
							EdgeIdxOfPositions.RemoveAt(1);
							MovedFlagOfPositions.RemoveAt(1);

							// 追加したものが即削除された場合ももう追加判定はとめて次のパーティクルに進む。
							// 追加と即削除を繰り返すとParticleIdx=0のままでループが進まないので。
							bExistAddedParticle = false;
						}
					}

					if (!bExistAddedParticle)
					{
						PrevPositions[0] = Positions[0];
					}
				}
				else if (ParticleIdx == (Positions.Num() - 1))
				{
					// 動いている頂点
					const FVector& TriVert1 = Positions[ParticleIdx];
					const FVector& TriVert2 = PrevPositions[ParticleIdx];
					
					// 固定している頂点は始点の判定と違ってPrevPositonsでなくPositionsなのに注意。
					// こうしないと1フレームでParticlesIdxの頂点が大きく動いたとき交差検出が漏れるケースがある
					// https://www.gdcvault.com/play/1027351/Rope-Simulation-in-Uncharted-4
					// の32分ごろの例。
					const FVector& TriVert0 = Positions[ParticleIdx - 1];

					// TODO: 上のifブロックと処理が冗長
					double NearestEdgeDistanceSq = DBL_MAX;
					FVector NearestIntersectPoint = FVector::ZeroVector;
					int32 NearestEdgeIdx = INDEX_NONE;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						// 異なるエッジでないものは採用しない
						if (EdgeIdxOfPositions[ParticleIdx - 1] == EdgeIdx || EdgeIdxOfPositions[ParticleIdx] == EdgeIdx)
						{
							continue;
						}

						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& RayStart = Edge.Key;
						const FVector& RayEnd = Edge.Value;
						FVector IntersectPoint;
						FVector IntersectNormal;
						bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
						if (bIntersecting)
						{
							// 元の線分と最も近いエッジを採用
							double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(IntersectPoint, TriVert0, TriVert2);
							if (EdgeDistanceSq < NearestEdgeDistanceSq)
							{
								// 等距離なら最も若いインデックスを採用する
								NearestEdgeDistanceSq = EdgeDistanceSq;
								NearestIntersectPoint = IntersectPoint;
								NearestEdgeIdx = EdgeIdx;
							}
						}
					}

					if (NearestEdgeIdx != INDEX_NONE)
					{
						// TODO:GDC動画のように延長線上を新たな終点におくと、同一平面に複数エッジがある
						// シェイプだと、同一平面上のエッジを拾い損ねることがFMath::SegmentTriangleIntersection()では簡単に起きる。
						// Triangleの辺にエッジがある場合に交差を検出失敗する。
						// よってPrevPositionsの更新は頂点の追加が終わるまでしない
						PrevPositions.Insert(NearestIntersectPoint, ParticleIdx);
						Positions.Insert(NearestIntersectPoint, ParticleIdx);
						EdgeIdxOfPositions.Insert(NearestEdgeIdx, ParticleIdx);
						MovedFlagOfPositions.Insert(true, ParticleIdx); // ここでtrueにして次イテレーションで最短コンストレイントを行う // TODO:逆順ループを作れば次イテレーションにしなくてもよくなり収束も早くなるかも

						// MoveHalfwayPositionからPositions[ParticleIdx]の間で動かしてさらに他のエッジ接触がないかチェックする
						bExistAddedParticle = true;
					}

					// エッジからはがす削除
					// TODO:終点と処理が冗長
					if (Positions.Num() >= 3)
					{
						// 0-2の線分の削除だけで判定していないのは、なにかに1をひっかけている状態で
						// 0-2を大きく動かすと0-2線分上にコリジョンがいない状態になり削除対象になってしまうから
						// TODO:本当は3つのライントレースでなくTriangleとコリジョンのOverlapをとりたかったがAPIがなかった
						// TODO:もっといいやり方あるかも

						bool bTraceComplex = false;
						FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraint), bTraceComplex);
						// Toleranceだけ0と2を双方から縮めるのは、どちらかあるいは両方がエッジと接触
						// していたらそのエッジとヒット判定になり、1の削除判定をしたいのにできないため
						// TODO:もっといい方法ある？
						// Toleranceだけ1を0-2の線分側に近づけるのは、1がエッジと接触してると接触判定し続けるため
						// TODO:もっといい方法ある？
						const FVector& SmallTriVert0 = Positions[Positions.Num() - 1] + (Positions[Positions.Num() - 3] - Positions[Positions.Num() - 1]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert1 = Positions[Positions.Num() - 1 - 1] + ((Positions[Positions.Num() - 1] + Positions[Positions.Num() - 3]) * 0.5 - Positions[Positions.Num() - 2]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert2 = Positions[Positions.Num() - 3] + (Positions[Positions.Num() - 1] - Positions[Positions.Num() - 3]).GetSafeNormal() * Tolerance;

						const FVector& SmallTriVertWS0 = GetActorTransform().TransformPosition(SmallTriVert0);
						const FVector& SmallTriVertWS1 = GetActorTransform().TransformPosition(SmallTriVert1);
						const FVector& SmallTriVertWS2 = GetActorTransform().TransformPosition(SmallTriVert2);

						bool bIntersectionExist = false;
						for (UPrimitiveComponent* Primitive : OverlapPrimitives)
						{
							FHitResult HitResult;
							bool bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS0, SmallTriVertWS1, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS0, bHit ? HitResult.Location : SmallTriVertWS1, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS1, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS1, SmallTriVertWS2, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS1, bHit ? HitResult.Location : SmallTriVertWS2, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS2, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS2, SmallTriVertWS0, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS2, bHit ? HitResult.Location : SmallTriVertWS0, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS0, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}
						}

						if (!bIntersectionExist)
						{
							int32 LastIdx = Positions.Num() - 2;
							PrevPositions.RemoveAt(LastIdx);
							Positions.RemoveAt(LastIdx);
							EdgeIdxOfPositions.RemoveAt(LastIdx);
							MovedFlagOfPositions.RemoveAt(LastIdx);

							bExistAddedParticle = false;
						}
					}

					if (!bExistAddedParticle)
					{
						PrevPositions[Positions.Num() - 1] = Positions[Positions.Num() - 1];
					}
				}
				else
				{
					// エッジ端に到達した場合
					// 頂点周辺のエッジの状況を調査し、エッジ移動、頂点追加/削除の判定を行う
					// TODO:今まで削除は始点と端点側のブロックでTriangleの3つのエッジとPrimitiveの交差判定でやってきたが本来はここでやるべき

					// 頂点を中心としたTolerance半径の球と接触するエッジを収集する
					TArray<int32> IntersectedEdgeIndices;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& EdgeStart = Edge.Key;
						const FVector& EdgeEnd = Edge.Value;

						double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(Positions[ParticleIdx], EdgeStart, EdgeEnd);
						if (EdgeDistanceSq < ToleranceSquared)
						{
							IntersectedEdgeIndices.Add(EdgeIdx);
						}
					}

					if (IntersectedEdgeIndices.Num() == 1)
					{
						// 検出したエッジは現在所属していエッジであるはず
						check(IntersectedEdgeIndices[0] == EdgeIdxOfPositions[ParticleIdx]);
						// メッシュとしてエッジ端に他のエッジが全く接してないのは許容できない
						check(false);
					}
					else if (IntersectedEdgeIndices.Num() > 1)
					{
						// エッジのペア配列を作る
						TArray<TPair<int32, int32>> EdgePairs;
						EdgePairs.SetNum(IntersectedEdgeIndices.Num() * (IntersectedEdgeIndices.Num() - 1) / 2); // n_C_2

						int32 PairIdx = 0;
						for (int32 EdgeIdx = 0; EdgeIdx < IntersectedEdgeIndices.Num() - 1; EdgeIdx++)
						{
							for (int32 AnotherEdgeIdx = EdgeIdx + 1; AnotherEdgeIdx < IntersectedEdgeIndices.Num(); AnotherEdgeIdx++)
							{
								EdgePairs[PairIdx] = TPair<int32, int32>(IntersectedEdgeIndices[EdgeIdx], IntersectedEdgeIndices[AnotherEdgeIdx]);
								PairIdx++;
							}
						}

						// TODO:頂点間がToleranceより近い場合は頂点をまとめるべきなのでこのTolerance処理は必要ない
						const FVector& PreSegmentDir = (Positions[ParticleIdx] - Positions[ParticleIdx - 1]).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると内積判定がおかしくなるのでとりあえず
						const FVector& PostSegmentDir = (Positions[ParticleIdx + 1] - Positions[ParticleIdx]).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると内積判定がおかしくなるのでとりあえず
						double DotProduct = FVector::DotProduct(PreSegmentDir, PostSegmentDir);

						FVector ParticleNormal = FVector::ZAxisVector;
						if (DotProduct > 1.0f - Tolerance) // 前セグメントと次セグメントが平行 // TODO:内積に同じTolerance使うのはよくないかも。次元が違うし
						{
							// 前セグメントと平行でないセグメントを探してそれを採用
							// TODO:後ろ方向にしか探してないので偏りがある
							for (int32 PostParticleIdx = ParticleIdx + 1; PostParticleIdx < Positions.Num(); PostParticleIdx++)
							{
								// TODO:頂点間がToleranceより近い場合は頂点をまとめるべきなのでこのTolerance処理は必要ない
								const FVector& NextDir = (Positions[PostParticleIdx] - Positions[ParticleIdx]).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると内積判定がおかしくなるのでとりあえず
								double NextDotProduct = FVector::DotProduct(PreSegmentDir, NextDir);
								if (FMath::Abs(NextDotProduct) < 1.0f - Tolerance)
								{
									// 頂点位置から、前頂点と後頂点を通る直線に下ろした垂線の足
									const FVector& DropFoot = FMath::ClosestPointOnInfiniteLine(Positions[ParticleIdx - 1], Positions[PostParticleIdx], Positions[ParticleIdx]);
									ParticleNormal = (Positions[ParticleIdx] - DropFoot).GetSafeNormal();
									break;
								}
							}
						}
						else if (-DotProduct > 1.0f - Tolerance) // 前セグメントと次セグメントが180度逆。
						{
							
							ParticleNormal = PreSegmentDir;
						}
						else
						{
							// 頂点位置から、前頂点と後頂点を通る直線に下ろした垂線の足
							const FVector& DropFoot = FMath::ClosestPointOnInfiniteLine(Positions[ParticleIdx - 1], Positions[ParticleIdx + 1], Positions[ParticleIdx]);
							ParticleNormal = (Positions[ParticleIdx] - DropFoot).GetSafeNormal();
						}

					}
				}
			}
			else
			{
				check(!bExistAddedParticle);
				PrevPositions[ParticleIdx] = Positions[ParticleIdx];
			}
		}

		bExistMovedParticle = false;
		for (int32 ParticleIdx = 0; ParticleIdx < Positions.Num() - 1; ParticleIdx++)
		{
			if (MovedFlagOfPositions[ParticleIdx])
			{
				bExistMovedParticle = true;
				break;
			}
		}
	}
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


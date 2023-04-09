// Copyright Epic Games, Inc. All Rights Reserved.

#include "UE5NiagaraSandboxGameMode.h"
#include "UE5NiagaraSandboxCharacter.h"
#include "UObject/ConstructorHelpers.h"

AUE5NiagaraSandboxGameMode::AUE5NiagaraSandboxGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}

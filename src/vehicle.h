// SPDX-FileCopyrightText: 2026 Nilo Technologies
// SPDX-License-Identifier: MIT

#pragma once

#include "container.h"

#include "box3d/vehicle.h"

typedef struct b3World b3World;

typedef struct b3VehicleWheel
{
	b3VehicleWheelDef def;

	b3JointId jointId;

	// Body the wheel joint attaches to: the world's static anchor or a non-static ground body
	b3BodyId groundBodyId;

	float sprungMass;
	float stiffness;
	float compressionDamping;
	float reboundDamping;

	float omega;
	float rotationAngle;
	float steerAngle;
	float stickyTimer;

	// Last cast and solve results
	b3Pos contactPoint;
	b3Vec3 contactNormal;
	b3ShapeId groundShapeId;
	uint64_t userMaterialId;
	float suspensionLength;
	float load;
	float longitudinalForce;
	float lateralForce;
	float slipRatio;
	float slipAngle;
	float driveTorque;
	float brakeTorque;
	bool hasContact;
	bool isSticky;
} b3VehicleWheel;

typedef struct b3Vehicle
{
	int id;
	uint16_t generation;

	b3BodyId chassisId;
	void* userData;

	b3VehicleChassis chassis;
	b3VehicleWheel wheels[B3_MAX_VEHICLE_WHEELS];
	int wheelCount;
	b3VehicleEngine engine;
	b3VehicleTransmission transmission;
	b3VehicleDrivetrain drivetrain;
	b3VehicleAntiRoll antiRoll;
	b3VehicleSteering steering;
	b3VehicleAssists assists;

	b3VehicleInput input;

	// Mass properties computed from the chassis shapes when the vehicle was created
	float baseMass;
	b3Vec3 baseCenter;
	b3Matrix3 baseInertia;

	float engineSpeed;
	int gear;
	float shiftTimer;
	float clutchTimer;
	float latencyTimer;
	float steer;
	float stuckTimer;
	float speed;
	float throttle;
	float brake;
	int wheelsInContact;
	bool stuck;
	bool massDirty;

	// Launch control: active flag, and how long the engine has gone without being held back
	bool isLaunching;
	float launchTimer;
} b3Vehicle;

b3DeclareArray( b3Vehicle );

void b3UpdateVehiclesPreStep( b3World* world, float timeStep );
void b3DestroyVehiclesOnBody( b3World* world, b3BodyId bodyId );

// SPDX-FileCopyrightText: 2026 Nilo Technologies
// SPDX-License-Identifier: MIT

#pragma once

#include "container.h"

#include "box3d/wheeled_vehicle.h"

typedef struct b3World b3World;

typedef struct b3WheeledVehicleWheel
{
	b3WheeledVehicleWheelDef def;

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
} b3WheeledVehicleWheel;

typedef struct b3WheeledVehicle
{
	int id;
	uint16_t generation;

	b3BodyId chassisId;
	void* userData;

	b3WheeledVehicleChassis chassis;
	b3WheeledVehicleWheel wheels[B3_MAX_VEHICLE_WHEELS];
	int wheelCount;
	b3WheeledVehicleEngine engine;
	b3WheeledVehicleTurbo turbo;
	b3WheeledVehicleTransmission transmission;
	b3WheeledVehicleDrivetrain drivetrain;
	b3WheeledVehicleAntiRoll antiRoll;
	b3WheeledVehicleSteering steering;
	b3WheeledVehicleAssists assists;

	b3WheeledVehicleInput input;

	// Mass properties computed from the chassis shapes when the vehicle was created
	float baseMass;
	b3Vec3 baseCenter;
	b3Matrix3 baseInertia;

	float engineSpeed;

	// Per turbo stage, 0 to 1 of the stage gain: the shaft spool, which spins down slowly, and the
	// delivered manifold pressure, which dumps fast when the throttle closes
	float turboSpool[B3_MAX_VEHICLE_TURBO_STAGES];
	float turboPressure[B3_MAX_VEHICLE_TURBO_STAGES];
	float boost;

	int gear;
	float shiftTimer;
	float clutchTimer;
	float latencyTimer;
	float steer;
	float stuckTimer;
	// The side self righting rolls toward near the roof (+1 or -1), or 0 when not yet chosen
	float rightingSide;
	// Self righting has started and holds until the car is back on its side or wheels
	bool righting;
	float speed;
	float throttle;
	float brake;
	int wheelsInContact;
	bool stuck;
	bool massDirty;

	// Launch control: active flag, whether a finished launch waits for the driver to lift, and how
	// long the engine has gone without being held back
	bool isLaunching;
	bool launchSpent;
	float launchTimer;
} b3WheeledVehicle;

b3DeclareArray( b3WheeledVehicle );

void b3UpdateWheeledVehiclesPreStep( b3World* world, float timeStep );
void b3DestroyWheeledVehiclesOnBody( b3World* world, b3BodyId bodyId );

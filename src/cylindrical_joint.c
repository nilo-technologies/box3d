// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "math_internal.h"
#include "physics_world.h"
#include "recording.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"

// A cylindrical joint is a prismatic joint with one rotation freed. Body B rotates about the x-axis of
// joint frame A and slides along the slide axis, which is the x-axis by default (coaxial) but may be any
// direction in frame A. Below, "axis" is the slide axis for the linear rows and the x-axis of frame A
// for the angular rows.
//
// Linear point-to-line constraint (same as the prismatic joint)
// d = pB - pA
// Cy = dot(perpY, d), Cz = dot(perpZ, d)
// Jy = [-perpY, -cross(d + rA, perpY), perpY, cross(rB, perpY)]
//
// Axial linear spring/motor/limit along the axis (same as the prismatic joint)
// C = dot(axis, d)
// J = [-axis, -cross(d + rA, axis), axis, cross(rB, axis)]
//
// Angular coaxial constraint (the revolute collinearity constraint with x as the free axis)
// qRel = conj(frameA) * frameB
// C = [qRel.y; qRel.z]
// Jy = 0.5 * rotate(frameA, s * ey + cross(v, ey))
// Jz = 0.5 * rotate(frameA, s * ez + cross(v, ez))
//
// Axial angular spring/motor/limit about the axis
// Cdot = dot(wB - wA, axis)
// K = dot(axis, (invIA + invIB) * axis)

// Twist angle about the x-axis in [-pi, pi]
static float b3GetTwistAngleX( b3Quat q )
{
	float twist = q.s < 0.0f ? b3Atan2( -q.v.x, -q.s ) : b3Atan2( q.v.x, q.s );
	return 2.0f * twist;
}

static b3Quat b3GetCylindricalRelativeRotation( b3Quat quatA, b3Quat quatB )
{
	if ( b3DotQuat( quatA, quatB ) < 0.0f )
	{
		// this keeps the twist angle in the range [-pi, pi]
		quatB = b3NegateQuat( quatB );
	}

	return b3InvMulQuat( quatA, quatB );
}

void b3CylindricalJoint_EnableSpring( b3JointId jointId, bool enableSpring )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointEnableSpring, jointId, enableSpring );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	if ( enableSpring != base->cylindricalJoint.enableSpring )
	{
		base->cylindricalJoint.springImpulse = 0.0f;
	}
	base->cylindricalJoint.enableSpring = enableSpring;
}

bool b3CylindricalJoint_IsSpringEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.enableSpring;
}

void b3CylindricalJoint_SetSpringHertz( b3JointId jointId, float hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && hertz >= 0.0f );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetSpringHertz, jointId, hertz );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.hertz = hertz;
}

float b3CylindricalJoint_GetSpringHertz( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.hertz;
}

void b3CylindricalJoint_SetSpringDampingRatio( b3JointId jointId, float dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetSpringDampingRatio, jointId, dampingRatio );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.dampingRatio = dampingRatio;
}

float b3CylindricalJoint_GetSpringDampingRatio( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.dampingRatio;
}

void b3CylindricalJoint_SetTargetTranslation( b3JointId jointId, float targetTranslation )
{
	B3_ASSERT( b3IsValidFloat( targetTranslation ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetTargetTranslation, jointId, targetTranslation );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.targetTranslation = targetTranslation;
}

float b3CylindricalJoint_GetTargetTranslation( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.targetTranslation;
}

void b3CylindricalJoint_EnableLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointEnableLimit, jointId, enableLimit );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	if ( enableLimit != base->cylindricalJoint.enableLimit )
	{
		base->cylindricalJoint.lowerImpulse = 0.0f;
		base->cylindricalJoint.upperImpulse = 0.0f;
	}
	base->cylindricalJoint.enableLimit = enableLimit;
}

bool b3CylindricalJoint_IsLimitEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.enableLimit;
}

float b3CylindricalJoint_GetLowerLimit( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.lowerTranslation;
}

float b3CylindricalJoint_GetUpperLimit( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.upperTranslation;
}

void b3CylindricalJoint_SetLimits( b3JointId jointId, float lower, float upper )
{
	B3_ASSERT( b3IsValidFloat( lower ) && b3IsValidFloat( upper ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetLimits, jointId, lower, upper );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.lowerTranslation = b3MinFloat( lower, upper );
	base->cylindricalJoint.upperTranslation = b3MaxFloat( lower, upper );
}

void b3CylindricalJoint_EnableMotor( b3JointId jointId, bool enableMotor )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointEnableMotor, jointId, enableMotor );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	if ( enableMotor != base->cylindricalJoint.enableMotor )
	{
		base->cylindricalJoint.motorImpulse = 0.0f;
	}
	base->cylindricalJoint.enableMotor = enableMotor;
}

bool b3CylindricalJoint_IsMotorEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.enableMotor;
}

void b3CylindricalJoint_SetMotorSpeed( b3JointId jointId, float motorSpeed )
{
	B3_ASSERT( b3IsValidFloat( motorSpeed ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetMotorSpeed, jointId, motorSpeed );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.motorSpeed = motorSpeed;
}

float b3CylindricalJoint_GetMotorSpeed( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.motorSpeed;
}

void b3CylindricalJoint_SetMaxMotorForce( b3JointId jointId, float force )
{
	B3_ASSERT( b3IsValidFloat( force ) && force >= 0.0f );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetMaxMotorForce, jointId, force );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.maxMotorForce = force;
}

float b3CylindricalJoint_GetMaxMotorForce( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.maxMotorForce;
}

float b3CylindricalJoint_GetMotorForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return world->inv_h * base->cylindricalJoint.motorImpulse;
}

void b3CylindricalJoint_EnableAngularSpring( b3JointId jointId, bool enableSpring )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointEnableAngularSpring, jointId, enableSpring );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	if ( enableSpring != base->cylindricalJoint.enableAngularSpring )
	{
		base->cylindricalJoint.angularSpringImpulse = 0.0f;
	}
	base->cylindricalJoint.enableAngularSpring = enableSpring;
}

bool b3CylindricalJoint_IsAngularSpringEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.enableAngularSpring;
}

void b3CylindricalJoint_SetAngularSpringHertz( b3JointId jointId, float hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && hertz >= 0.0f );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetAngularSpringHertz, jointId, hertz );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.angularHertz = hertz;
}

float b3CylindricalJoint_GetAngularSpringHertz( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.angularHertz;
}

void b3CylindricalJoint_SetAngularSpringDampingRatio( b3JointId jointId, float dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetAngularSpringDampingRatio, jointId, dampingRatio );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.angularDampingRatio = dampingRatio;
}

float b3CylindricalJoint_GetAngularSpringDampingRatio( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.angularDampingRatio;
}

void b3CylindricalJoint_SetTargetAngle( b3JointId jointId, float targetAngle )
{
	B3_ASSERT( b3IsValidFloat( targetAngle ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetTargetAngle, jointId, targetAngle );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.targetAngle = b3ClampFloat( targetAngle, -B3_PI, B3_PI );
}

float b3CylindricalJoint_GetTargetAngle( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.targetAngle;
}

void b3CylindricalJoint_EnableAngularLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointEnableAngularLimit, jointId, enableLimit );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	if ( enableLimit != base->cylindricalJoint.enableAngularLimit )
	{
		base->cylindricalJoint.lowerAngularImpulse = 0.0f;
		base->cylindricalJoint.upperAngularImpulse = 0.0f;
	}
	base->cylindricalJoint.enableAngularLimit = enableLimit;
}

bool b3CylindricalJoint_IsAngularLimitEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.enableAngularLimit;
}

float b3CylindricalJoint_GetLowerAngle( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.lowerAngle;
}

float b3CylindricalJoint_GetUpperAngle( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.upperAngle;
}

void b3CylindricalJoint_SetAngularLimits( b3JointId jointId, float lower, float upper )
{
	B3_ASSERT( b3IsValidFloat( lower ) && b3IsValidFloat( upper ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetAngularLimits, jointId, lower, upper );
	float lowerAngle = b3MinFloat( lower, upper );
	float upperAngle = b3MaxFloat( lower, upper );

	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.lowerAngle = b3ClampFloat( lowerAngle, -0.99f * B3_PI, 0.99f * B3_PI );
	base->cylindricalJoint.upperAngle = b3ClampFloat( upperAngle, -0.99f * B3_PI, 0.99f * B3_PI );
}

void b3CylindricalJoint_EnableAngularMotor( b3JointId jointId, bool enableMotor )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointEnableAngularMotor, jointId, enableMotor );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	if ( enableMotor != base->cylindricalJoint.enableAngularMotor )
	{
		base->cylindricalJoint.angularMotorImpulse = 0.0f;
	}
	base->cylindricalJoint.enableAngularMotor = enableMotor;
}

bool b3CylindricalJoint_IsAngularMotorEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.enableAngularMotor;
}

void b3CylindricalJoint_SetAngularMotorSpeed( b3JointId jointId, float motorSpeed )
{
	B3_ASSERT( b3IsValidFloat( motorSpeed ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetAngularMotorSpeed, jointId, motorSpeed );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.angularMotorSpeed = motorSpeed;
}

float b3CylindricalJoint_GetAngularMotorSpeed( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.angularMotorSpeed;
}

void b3CylindricalJoint_SetMaxMotorTorque( b3JointId jointId, float torque )
{
	B3_ASSERT( b3IsValidFloat( torque ) && torque >= 0.0f );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, CylindricalJointSetMaxMotorTorque, jointId, torque );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	base->cylindricalJoint.maxMotorTorque = torque;
}

float b3CylindricalJoint_GetMaxMotorTorque( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return base->cylindricalJoint.maxMotorTorque;
}

float b3CylindricalJoint_GetMotorTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	return world->inv_h * base->cylindricalJoint.angularMotorImpulse;
}

float b3CylindricalJoint_GetTranslation( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );

	b3Quat slideQ = b3MulQuat( transformA.q, b3MulQuat( base->localFrameA.q, base->cylindricalJoint.slideRotation ) );
	b3Vec3 jointAxis = b3RotateVector( slideQ, b3Vec3_axisX );
	b3Vec3 anchorA = b3RotateVector( transformA.q, base->localFrameA.p );
	b3Vec3 anchorB = b3RotateVector( transformB.q, base->localFrameB.p );
	b3Vec3 d = b3Add( b3SubPos( transformB.p, transformA.p ), b3Sub( anchorB, anchorA ) );
	return b3Dot( d, jointAxis );
}

float b3CylindricalJoint_GetSpeed( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );

	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );
	b3BodySim* bodySimA = b3GetBodySim( world, bodyA );
	b3BodySim* bodySimB = b3GetBodySim( world, bodyB );
	b3BodyState* stateA = b3GetBodyState( world, bodyA );
	b3BodyState* stateB = b3GetBodyState( world, bodyB );

	b3Quat qA = bodySimA->transform.q;
	b3Quat qB = bodySimB->transform.q;

	b3Vec3 axisA = b3RotateVector( b3MulQuat( qA, b3MulQuat( base->localFrameA.q, base->cylindricalJoint.slideRotation ) ),
								   b3Vec3_axisX );
	b3Vec3 rA = b3RotateVector( qA, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	b3Vec3 rB = b3RotateVector( qB, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );
	b3Vec3 d = b3Add( b3SubPos( bodySimB->center, bodySimA->center ), b3Sub( rB, rA ) );

	b3Vec3 vA = stateA ? stateA->linearVelocity : b3Vec3_zero;
	b3Vec3 vB = stateB ? stateB->linearVelocity : b3Vec3_zero;
	b3Vec3 wA = stateA ? stateA->angularVelocity : b3Vec3_zero;
	b3Vec3 wB = stateB ? stateB->angularVelocity : b3Vec3_zero;

	b3Vec3 vRel = b3Sub( b3Add( vB, b3Cross( wB, rB ) ), b3Add( vA, b3Cross( wA, rA ) ) );

	// The axis moves with body A, so account for its rotation.
	return b3Dot( d, b3Cross( wA, axisA ) ) + b3Dot( axisA, vRel );
}

float b3CylindricalJoint_GetAngle( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );

	b3Quat quatA = b3MulQuat( transformA.q, base->localFrameA.q );
	b3Quat quatB = b3MulQuat( transformB.q, base->localFrameB.q );
	return b3GetTwistAngleX( b3GetCylindricalRelativeRotation( quatA, quatB ) );
}

float b3CylindricalJoint_GetAngularSpeed( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_cylindricalJoint );

	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );
	b3BodySim* bodySimA = b3GetBodySim( world, bodyA );
	b3BodyState* stateA = b3GetBodyState( world, bodyA );
	b3BodyState* stateB = b3GetBodyState( world, bodyB );

	b3Vec3 axisA = b3RotateVector( bodySimA->transform.q, b3RotateVector( base->localFrameA.q, b3Vec3_axisX ) );
	b3Vec3 wA = stateA ? stateA->angularVelocity : b3Vec3_zero;
	b3Vec3 wB = stateB ? stateB->angularVelocity : b3Vec3_zero;
	return b3Dot( b3Sub( wB, wA ), axisA );
}

b3Vec3 b3GetCylindricalJointForce( b3World* world, b3JointSim* base )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3CylindricalJoint* joint = &base->cylindricalJoint;

	// impulse in joint space
	b3Vec3 impulse = {
		joint->springImpulse + joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse,
		joint->perpImpulse.x,
		joint->perpImpulse.y,
	};

	b3Vec3 force = b3MulSV( world->inv_h, impulse );
	force = b3RotateVector( b3MulQuat( base->localFrameA.q, joint->slideRotation ), force );
	return b3RotateVector( transformA.q, force );
}

b3Vec3 b3GetCylindricalJointTorque( b3World* world, b3JointSim* base )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3CylindricalJoint* joint = &base->cylindricalJoint;

	// The perpendicular rows use half-angle Jacobians, so scale them back to torque
	b3Vec3 impulse = {
		joint->angularSpringImpulse + joint->angularMotorImpulse + joint->lowerAngularImpulse - joint->upperAngularImpulse,
		0.5f * joint->angularImpulse.x,
		0.5f * joint->angularImpulse.y,
	};

	b3Vec3 torque = b3MulSV( world->inv_h, impulse );
	torque = b3RotateVector( base->localFrameA.q, torque );
	return b3RotateVector( transformA.q, torque );
}

void b3PrepareCylindricalJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_cylindricalJoint );

	b3World* world = context->world;

	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );

	B3_ASSERT( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet );
	b3SolverSet* setA = b3Array_Get( world->solverSets, bodyA->setIndex );
	b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b3BodySim* bodySimA = b3Array_Get( setA->bodySims, localIndexA );
	b3BodySim* bodySimB = b3Array_Get( setB->bodySims, localIndexB );

	base->invMassA = bodySimA->invMass;
	base->invMassB = bodySimB->invMass;
	base->invIA = bodySimA->invInertiaWorld;
	base->invIB = bodySimB->invInertiaWorld;

	b3Matrix3 invInertiaSum = b3AddMM( base->invIA, base->invIB );
	base->fixedRotation = b3Det( invInertiaSum ) < 1000.0f * FLT_MIN;

	b3CylindricalJoint* joint = &base->cylindricalJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Joint anchor frames with world space rotation, relative to center of mass
	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	joint->deltaCenter = b3SubPos( bodySimB->center, bodySimA->center );

	b3Matrix3 matrixA = b3MakeMatrixFromQuat( joint->frameA.q );
	joint->rotationAxis = matrixA.cx;

	b3Matrix3 slideMatrix = b3MakeMatrixFromQuat( b3MulQuat( joint->frameA.q, joint->slideRotation ) );
	joint->slideAxis = slideMatrix.cx;
	joint->perpAxisY = slideMatrix.cy;
	joint->perpAxisZ = slideMatrix.cz;

	float k = b3Dot( joint->rotationAxis, b3MulMV( invInertiaSum, joint->rotationAxis ) );
	joint->axialAngularMass = k > 0.0f ? 1.0f / k : 0.0f;

	b3Quat relQ = b3GetCylindricalRelativeRotation( joint->frameA.q, joint->frameB.q );

	// Needed for warm starting
	joint->angularAxisY = b3MulSV(
		0.5f, b3RotateVector( joint->frameA.q, b3Add( b3MulSV( relQ.s, b3Vec3_axisY ), b3Cross( relQ.v, b3Vec3_axisY ) ) ) );
	joint->angularAxisZ = b3MulSV(
		0.5f, b3RotateVector( joint->frameA.q, b3Add( b3MulSV( relQ.s, b3Vec3_axisZ ), b3Cross( relQ.v, b3Vec3_axisZ ) ) ) );

	joint->springSoftness = b3MakeSoft( joint->hertz, joint->dampingRatio, context->h );
	joint->angularSpringSoftness = b3MakeSoft( joint->angularHertz, joint->angularDampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->perpImpulse = (b3Vec2){ 0.0f, 0.0f };
		joint->angularImpulse = (b3Vec2){ 0.0f, 0.0f };
		joint->springImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
		joint->angularSpringImpulse = 0.0f;
		joint->angularMotorImpulse = 0.0f;
		joint->lowerAngularImpulse = 0.0f;
		joint->upperAngularImpulse = 0.0f;
	}
}

void b3WarmStartCylindricalJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_cylindricalJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3CylindricalJoint* joint = &base->cylindricalJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
	b3Vec3 d = b3Add( b3Add( b3Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b3Sub( rB, rA ) );

	b3Vec3 jointAxis = b3RotateVector( stateA->deltaRotation, joint->slideAxis );
	b3Vec3 rotationAxis = b3RotateVector( stateA->deltaRotation, joint->rotationAxis );
	b3Vec3 perpY = b3RotateVector( stateA->deltaRotation, joint->perpAxisY );
	b3Vec3 perpZ = b3RotateVector( stateA->deltaRotation, joint->perpAxisZ );
	b3Vec3 dA = b3Add( rA, d );

	float axialImpulse = joint->springImpulse + joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;
	b3Vec2 perpImpulse = joint->perpImpulse;

	b3Vec3 P = b3Blend3( axialImpulse, jointAxis, perpImpulse.x, perpY, perpImpulse.y, perpZ );
	b3Vec3 LA = b3Cross( dA, P );
	b3Vec3 LB = b3Cross( rB, P );

	float axialAngularImpulse =
		joint->angularSpringImpulse + joint->angularMotorImpulse + joint->lowerAngularImpulse - joint->upperAngularImpulse;
	b3Vec3 L = b3Blend3( axialAngularImpulse, rotationAxis, joint->angularImpulse.x, joint->angularAxisY, joint->angularImpulse.y,
						 joint->angularAxisZ );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = b3MulSub( stateA->linearVelocity, mA, P );
		stateA->angularVelocity = b3Sub( stateA->angularVelocity, b3MulMV( iA, b3Add( LA, L ) ) );
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = b3MulAdd( stateB->linearVelocity, mB, P );
		stateB->angularVelocity = b3Add( stateB->angularVelocity, b3MulMV( iB, b3Add( LB, L ) ) );
	}
}

void b3SolveCylindricalJoint( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_cylindricalJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3CylindricalJoint* joint = &base->cylindricalJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	bool fixedRotation = base->fixedRotation;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
	b3Vec3 d = b3Add( b3Add( b3Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b3Sub( rB, rA ) );
	b3Vec3 dA = b3Add( rA, d );

	// Linear rows act along the slide axis, angular rows about the rotation axis
	b3Vec3 jointAxis = b3RotateVector( stateA->deltaRotation, joint->slideAxis );
	b3Vec3 rotationAxis = b3RotateVector( stateA->deltaRotation, joint->rotationAxis );
	b3Vec3 sAx = b3Cross( dA, jointAxis );
	b3Vec3 sBx = b3Cross( rB, jointAxis );
	float translation = b3Dot( d, jointAxis );

	// The axial effective mass must be fresh to avoid divergence when the joint is stressed
	float ka = mA + mB + b3Dot( sAx, b3MulMV( iA, sAx ) ) + b3Dot( sBx, b3MulMV( iB, sBx ) );
	float axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
	b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
	b3Quat relQ = b3GetCylindricalRelativeRotation( quatA, quatB );
	float angle = b3GetTwistAngleX( relQ );
	float axialAngularMass = joint->axialAngularMass;

	// Linear spring
	if ( joint->enableSpring )
	{
		float c = translation - joint->targetTranslation;
		float bias = joint->springSoftness.biasRate * c;
		float massScale = joint->springSoftness.massScale;
		float impulseScale = joint->springSoftness.impulseScale;

		b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, dA ) );
		float cdot = b3Dot( vRel, jointAxis );
		float deltaImpulse = -massScale * axialMass * ( cdot + bias ) - impulseScale * joint->springImpulse;
		joint->springImpulse += deltaImpulse;

		b3Vec3 P = b3MulSV( deltaImpulse, jointAxis );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, b3MulSV( deltaImpulse, sAx ) ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, b3MulSV( deltaImpulse, sBx ) ) );
	}

	// Linear motor
	if ( joint->enableMotor )
	{
		b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, dA ) );
		float cdot = b3Dot( vRel, jointAxis ) - joint->motorSpeed;

		float deltaImpulse = -axialMass * cdot;
		float newImpulse = joint->motorImpulse + deltaImpulse;
		float maxImpulse = joint->maxMotorForce * context->h;
		newImpulse = b3ClampFloat( newImpulse, -maxImpulse, maxImpulse );
		deltaImpulse = newImpulse - joint->motorImpulse;
		joint->motorImpulse = newImpulse;

		b3Vec3 P = b3MulSV( deltaImpulse, jointAxis );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, b3MulSV( deltaImpulse, sAx ) ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, b3MulSV( deltaImpulse, sBx ) ) );
	}

	// Linear limit
	if ( joint->enableLimit )
	{
		float speculativeDistance = 0.25f * ( joint->upperTranslation - joint->lowerTranslation );

		// Lower limit
		{
			float C = translation - joint->lowerTranslation;

			if ( C < speculativeDistance )
			{
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( C > 0.0f )
				{
					// speculation
					bias = C * context->inv_h;
				}
				else if ( useBias )
				{
					bias = base->constraintSoftness.biasRate * C;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}

				b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, dA ) );
				float cdot = b3Dot( vRel, jointAxis );
				float oldImpulse = joint->lowerImpulse;
				float deltaImpulse = -massScale * axialMass * ( cdot + bias ) - impulseScale * oldImpulse;
				joint->lowerImpulse = b3MaxFloat( oldImpulse + deltaImpulse, 0.0f );
				deltaImpulse = joint->lowerImpulse - oldImpulse;

				b3Vec3 P = b3MulSV( deltaImpulse, jointAxis );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3MulSV( deltaImpulse, sAx ) ) );
				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3MulSV( deltaImpulse, sBx ) ) );
			}
			else
			{
				joint->lowerImpulse = 0.0f;
			}
		}

		// Upper limit
		{
			float C = joint->upperTranslation - translation;

			if ( C < speculativeDistance )
			{
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( C > 0.0f )
				{
					// speculation
					bias = C * context->inv_h;
				}
				else if ( useBias )
				{
					bias = base->constraintSoftness.biasRate * C;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}

				// sign flipped on Cdot
				b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, dA ) );
				float cdot = -b3Dot( vRel, jointAxis );
				float oldImpulse = joint->upperImpulse;
				float deltaImpulse = -massScale * axialMass * ( cdot + bias ) - impulseScale * oldImpulse;
				joint->upperImpulse = b3MaxFloat( oldImpulse + deltaImpulse, 0.0f );

				// sign flipped on applied impulse
				float negDeltaImpulse = oldImpulse - joint->upperImpulse;
				b3Vec3 P = b3MulSV( negDeltaImpulse, jointAxis );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3MulSV( negDeltaImpulse, sAx ) ) );
				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3MulSV( negDeltaImpulse, sBx ) ) );
			}
			else
			{
				joint->upperImpulse = 0.0f;
			}
		}
	}

	if ( fixedRotation == false )
	{
		// Angular spring
		if ( joint->enableAngularSpring )
		{
			float c = angle - joint->targetAngle;
			float bias = joint->angularSpringSoftness.biasRate * c;
			float massScale = joint->angularSpringSoftness.massScale;
			float impulseScale = joint->angularSpringSoftness.impulseScale;
			float cdot = b3Dot( b3Sub( wB, wA ), rotationAxis );

			float deltaImpulse =
				-massScale * axialAngularMass * ( cdot + bias ) - impulseScale * joint->angularSpringImpulse;
			joint->angularSpringImpulse += deltaImpulse;

			wA = b3MulSub( wA, deltaImpulse, b3MulMV( iA, rotationAxis ) );
			wB = b3MulAdd( wB, deltaImpulse, b3MulMV( iB, rotationAxis ) );
		}

		// Angular motor
		if ( joint->enableAngularMotor )
		{
			float cdot = b3Dot( b3Sub( wB, wA ), rotationAxis ) - joint->angularMotorSpeed;

			float deltaImpulse = -axialAngularMass * cdot;
			float newImpulse = joint->angularMotorImpulse + deltaImpulse;
			float maxImpulse = joint->maxMotorTorque * context->h;
			newImpulse = b3ClampFloat( newImpulse, -maxImpulse, maxImpulse );
			deltaImpulse = newImpulse - joint->angularMotorImpulse;
			joint->angularMotorImpulse = newImpulse;

			wA = b3MulSub( wA, deltaImpulse, b3MulMV( iA, rotationAxis ) );
			wB = b3MulAdd( wB, deltaImpulse, b3MulMV( iB, rotationAxis ) );
		}

		// Angular limit
		if ( joint->enableAngularLimit )
		{
			// Lower limit
			{
				float c = angle - joint->lowerAngle;
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( c > 0.0f )
				{
					// speculation
					bias = c * context->inv_h;
				}
				else if ( useBias )
				{
					bias = base->constraintSoftness.biasRate * c;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}

				float cdot = b3Dot( b3Sub( wB, wA ), rotationAxis );
				float oldImpulse = joint->lowerAngularImpulse;
				float deltaImpulse = -massScale * axialAngularMass * ( cdot + bias ) - impulseScale * oldImpulse;
				joint->lowerAngularImpulse = b3MaxFloat( oldImpulse + deltaImpulse, 0.0f );
				deltaImpulse = joint->lowerAngularImpulse - oldImpulse;

				wA = b3MulSub( wA, deltaImpulse, b3MulMV( iA, rotationAxis ) );
				wB = b3MulAdd( wB, deltaImpulse, b3MulMV( iB, rotationAxis ) );
			}

			// Upper limit
			{
				float c = joint->upperAngle - angle;
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( c > 0.0f )
				{
					// speculation
					bias = c * context->inv_h;
				}
				else if ( useBias )
				{
					bias = base->constraintSoftness.biasRate * c;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}

				// sign flipped on Cdot
				float cdot = b3Dot( b3Sub( wA, wB ), rotationAxis );
				float oldImpulse = joint->upperAngularImpulse;
				float deltaImpulse = -massScale * axialAngularMass * ( cdot + bias ) - impulseScale * oldImpulse;
				joint->upperAngularImpulse = b3MaxFloat( oldImpulse + deltaImpulse, 0.0f );
				deltaImpulse = joint->upperAngularImpulse - oldImpulse;

				// sign flipped on applied impulse
				wA = b3MulAdd( wA, deltaImpulse, b3MulMV( iA, rotationAxis ) );
				wB = b3MulSub( wB, deltaImpulse, b3MulMV( iB, rotationAxis ) );
			}
		}

		// Coaxial constraint: lock the two rotations perpendicular to the axis
		{
			b3Vec2 bias = { 0.0f, 0.0f };
			float massScale = 1.0f;
			float impulseScale = 0.0f;

			if ( useBias )
			{
				b3Vec2 c = { relQ.v.y, relQ.v.z };
				bias = b3MulSV2( base->constraintSoftness.biasRate, c );
				massScale = base->constraintSoftness.massScale;
				impulseScale = base->constraintSoftness.impulseScale;
			}

			b3Vec3 axisY = b3MulSV(
				0.5f, b3RotateVector( quatA, b3Add( b3MulSV( relQ.s, b3Vec3_axisY ), b3Cross( relQ.v, b3Vec3_axisY ) ) ) );
			b3Vec3 axisZ = b3MulSV(
				0.5f, b3RotateVector( quatA, b3Add( b3MulSV( relQ.s, b3Vec3_axisZ ), b3Cross( relQ.v, b3Vec3_axisZ ) ) ) );
			joint->angularAxisY = axisY;
			joint->angularAxisZ = axisZ;

			b3Matrix3 invInertiaSum = b3AddMM( iA, iB );
			float kyy = b3Dot( axisY, b3MulMV( invInertiaSum, axisY ) );
			float kzz = b3Dot( axisZ, b3MulMV( invInertiaSum, axisZ ) );
			float kyz = b3Dot( axisY, b3MulMV( invInertiaSum, axisZ ) );
			b3Matrix2 K = { { kyy, kyz }, { kyz, kzz } };

			b3Vec3 wRel = b3Sub( wB, wA );
			b3Vec2 cdot = { b3Dot( wRel, axisY ), b3Dot( wRel, axisZ ) };
			b3Vec2 oldImpulse = joint->angularImpulse;
			b3Vec2 sol = b3Solve2( K, b3Add2( cdot, bias ) );
			b3Vec2 deltaImpulse = b3Sub2( b3MulSV2( -massScale, sol ), b3MulSV2( impulseScale, oldImpulse ) );
			joint->angularImpulse = b3Add2( oldImpulse, deltaImpulse );

			b3Vec3 L = b3Blend2( deltaImpulse.x, axisY, deltaImpulse.y, axisZ );
			wA = b3Sub( wA, b3MulMV( iA, L ) );
			wB = b3Add( wB, b3MulMV( iB, L ) );
		}
	}

	// Point-to-line constraint
	{
		b3Vec3 perpY = b3RotateVector( stateA->deltaRotation, joint->perpAxisY );
		b3Vec3 perpZ = b3RotateVector( stateA->deltaRotation, joint->perpAxisZ );

		b3Vec2 bias = { 0.0f, 0.0f };
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b3Vec2 c = { b3Dot( perpY, d ), b3Dot( perpZ, d ) };
			bias = b3MulSV2( base->constraintSoftness.biasRate, c );
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, dA ) );
		b3Vec2 cdot = { b3Dot( perpY, vRel ), b3Dot( perpZ, vRel ) };

		b3Vec3 sAy = b3Cross( dA, perpY );
		b3Vec3 sBy = b3Cross( rB, perpY );
		b3Vec3 sAz = b3Cross( dA, perpZ );
		b3Vec3 sBz = b3Cross( rB, perpZ );

		float kyy = mA + mB + b3Dot( sAy, b3MulMV( iA, sAy ) ) + b3Dot( sBy, b3MulMV( iB, sBy ) );
		float kyz = b3Dot( sAy, b3MulMV( iA, sAz ) ) + b3Dot( sBy, b3MulMV( iB, sBz ) );
		float kzz = mA + mB + b3Dot( sAz, b3MulMV( iA, sAz ) ) + b3Dot( sBz, b3MulMV( iB, sBz ) );
		b3Matrix2 K = { { kyy, kyz }, { kyz, kzz } };

		b3Vec2 oldImpulse = joint->perpImpulse;
		b3Vec2 sol = b3Solve2( K, b3Add2( cdot, bias ) );
		b3Vec2 deltaImpulse = b3Sub2( b3MulSV2( -massScale, sol ), b3MulSV2( impulseScale, oldImpulse ) );
		joint->perpImpulse = b3Add2( oldImpulse, deltaImpulse );

		b3Vec3 P = b3Blend2( deltaImpulse.x, perpY, deltaImpulse.y, perpZ );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, b3Blend2( deltaImpulse.x, sAy, deltaImpulse.y, sAz ) ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, b3Blend2( deltaImpulse.x, sBy, deltaImpulse.y, sBz ) ) );
	}

	B3_ASSERT( b3IsValidVec3( vA ) );
	B3_ASSERT( b3IsValidVec3( wA ) );
	B3_ASSERT( b3IsValidVec3( vB ) );
	B3_ASSERT( b3IsValidVec3( wB ) );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}

void b3DrawCylindricalJoint( b3DebugDraw* draw, b3JointSim* base, b3WorldTransform transformA, b3WorldTransform transformB,
							 float scale )
{
	b3WorldTransform frameA = b3MulWorldTransforms( transformA, base->localFrameA );
	b3WorldTransform frameB = b3MulWorldTransforms( transformB, base->localFrameB );

	b3CylindricalJoint* joint = &base->cylindricalJoint;
	b3Vec3 axis = b3RotateVector( b3MulQuat( frameA.q, joint->slideRotation ), b3Vec3_axisX );

	if ( joint->enableLimit )
	{
		b3Pos p1 = b3OffsetPos( frameA.p, b3MulSV( joint->lowerTranslation, axis ) );
		b3Pos p2 = b3OffsetPos( frameA.p, b3MulSV( joint->upperTranslation, axis ) );
		draw->DrawSegmentFcn( p1, p2, b3_colorOrange, draw->context );
		draw->DrawPointFcn( p1, 10.0f, b3_colorGreen, draw->context );
		draw->DrawPointFcn( p2, 10.0f, b3_colorRed, draw->context );
	}
	else
	{
		b3Pos p1 = b3OffsetPos( frameA.p, b3MulSV( -0.5f * scale, axis ) );
		b3Pos p2 = b3OffsetPos( frameA.p, b3MulSV( 0.5f * scale, axis ) );
		draw->DrawSegmentFcn( p1, p2, b3_colorOrange, draw->context );
	}

	// Frame B spokes show the rotation about the axis
	float s = 0.2f * scale;
	b3Matrix3 RB = b3MakeMatrixFromQuat( frameB.q );
	draw->DrawSegmentFcn( frameB.p, b3OffsetPos( frameB.p, b3MulSV( s, RB.cy ) ), b3_colorGreen, draw->context );
	draw->DrawSegmentFcn( frameB.p, b3OffsetPos( frameB.p, b3MulSV( s, RB.cz ) ), b3_colorBlue, draw->context );
	draw->DrawPointFcn( frameB.p, 8.0f, b3_colorViolet, draw->context );
}

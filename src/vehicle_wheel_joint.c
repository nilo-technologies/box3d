// SPDX-FileCopyrightText: 2026 Nilo Technologies
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "math_internal.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"

#include <stdio.h>

// Soft constraint coefficients from an angular frequency (rad/s) rather than hertz
static b3Softness b3MakeSoftOmega( float omega, float zeta, float h )
{
	float a1 = 2.0f * zeta + h * omega;
	float a2 = h * omega * a1;
	float a3 = 1.0f / ( 1.0f + a2 );
	return (b3Softness){
		.biasRate = omega / a1,
		.massScale = a2 * a3,
		.impulseScale = a3,
	};
}

b3Vec3 b3GetVehicleWheelJointForce( b3World* world, b3JointSim* base )
{
	b3VehicleWheelJoint* joint = &base->vehicleWheelJoint;
	b3Vec3 impulse = b3MulSV( joint->springImpulse + joint->bumpImpulse, joint->normal );
	impulse = b3MulAdd( impulse, joint->longitudinalImpulse, joint->forward );
	impulse = b3MulAdd( impulse, joint->lateralImpulse, joint->lateral );
	return b3MulSV( world->inv_h, impulse );
}

static float b3EffectiveMass( float mA, float mB, b3Matrix3 iA, b3Matrix3 iB, b3Vec3 rA, b3Vec3 rB, b3Vec3 axis,
							  float extra )
{
	b3Vec3 rnA = b3Cross( rA, axis );
	b3Vec3 rnB = b3Cross( rB, axis );
	float k = mA + mB + b3Dot( rnA, b3MulMV( iA, rnA ) ) + b3Dot( rnB, b3MulMV( iB, rnB ) ) + extra;
	return k > 0.0f ? 1.0f / k : 0.0f;
}

void b3PrepareVehicleWheelJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_vehicleWheelJoint );

	b3World* world = context->world;

	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );

	B3_ASSERT( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet );

	b3SolverSet* setA = b3Array_Get( world->solverSets, bodyA->setIndex );
	b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex );

	b3BodySim* bodySimA = b3Array_Get( setA->bodySims, bodyA->localIndex );
	b3BodySim* bodySimB = b3Array_Get( setB->bodySims, bodyB->localIndex );

	float mA = bodySimA->invMass;
	float mB = bodySimB->invMass;
	b3Matrix3 iA = bodySimA->invInertiaWorld;
	b3Matrix3 iB = bodySimB->invInertiaWorld;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b3VehicleWheelJoint* joint = &base->vehicleWheelJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? bodyA->localIndex : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? bodyB->localIndex : B3_NULL_INDEX;

	// Anchors relative to the center of mass, in world orientation
	joint->anchorA = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->anchorB = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	b3Vec3 rA = joint->anchorA;
	b3Vec3 rB = joint->anchorB;
	b3Vec3 rLatA = b3Add( rA, joint->rollOffset );
	float spinTerm = joint->radius * joint->radius * joint->invInertia;

	joint->normalMass = b3EffectiveMass( mA, mB, iA, iB, rA, rB, joint->normal, 0.0f );
	joint->longitudinalMass = b3EffectiveMass( mA, mB, iA, iB, rA, rB, joint->forward, spinTerm );
	joint->lockedMass = b3EffectiveMass( mA, mB, iA, iB, rA, rB, joint->forward, 0.0f );
	joint->lateralMass = b3EffectiveMass( mA, mB, iA, iB, rLatA, rB, joint->lateral, 0.0f );

	// The friction and brake rows are not warm started. Left and right wheels on an axle can
	// otherwise keep opposing lateral impulses alive forever, using up grip.
	joint->longitudinalImpulse = 0.0f;
	joint->lateralImpulse = 0.0f;
	joint->brakeImpulse = 0.0f;

	if ( context->enableWarmStarting == false || joint->hasContact == false )
	{
		joint->springImpulse = 0.0f;
		joint->bumpImpulse = 0.0f;
	}
}

static void b3ApplyWheelImpulse( b3VehicleWheelJoint* joint, float mA, float mB, b3Matrix3 iA, b3Matrix3 iB,
								 b3Vec3 rA, b3Vec3 rB, b3Vec3 impulse, b3Vec3* vA, b3Vec3* wA, b3Vec3* vB,
								 b3Vec3* wB )
{
	B3_UNUSED( joint );
	*vA = b3MulAdd( *vA, mA, impulse );
	*wA = b3Add( *wA, b3MulMV( iA, b3Cross( rA, impulse ) ) );
	*vB = b3MulSub( *vB, mB, impulse );
	*wB = b3Sub( *wB, b3MulMV( iB, b3Cross( rB, impulse ) ) );
}

void b3WarmStartVehicleWheelJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_vehicleWheelJoint );

	b3VehicleWheelJoint* joint = &base->vehicleWheelJoint;

	// The drivetrain torque was solved implicitly in the pre-step, so applying it here per
	// substep is just distributing a known impulse.
	joint->omega += context->h * joint->invInertia * joint->driveTorque;

	joint->longitudinalImpulse = 0.0f;
	joint->lateralImpulse = 0.0f;
	joint->brakeImpulse = 0.0f;

	if ( joint->hasContact == false )
	{
		return;
	}

	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->anchorA );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->anchorB );

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3Vec3 normalImpulse = b3MulSV( joint->springImpulse + joint->bumpImpulse, joint->normal );
	b3ApplyWheelImpulse( joint, mA, mB, iA, iB, rA, rB, normalImpulse, &vA, &wA, &vB, &wB );

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

void b3SolveVehicleWheelJoint( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_vehicleWheelJoint );

	b3VehicleWheelJoint* joint = &base->vehicleWheelJoint;
	float h = context->h;

	// Brake row on the wheel spin. With contact it is solved together with the roller row below.
	if ( joint->hasContact == false && joint->brakeTorque > 0.0f && joint->invInertia > 0.0f )
	{
		float maxImpulse = h * joint->brakeTorque;
		float impulse = -joint->omega / joint->invInertia;
		float oldImpulse = joint->brakeImpulse;
		joint->brakeImpulse = b3ClampFloat( oldImpulse + impulse, -maxImpulse, maxImpulse );
		joint->omega += joint->invInertia * ( joint->brakeImpulse - oldImpulse );
	}

	if ( joint->hasContact == false )
	{
		return;
	}

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->anchorA );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->anchorB );
	b3Vec3 rLatA = b3Add( rA, joint->rollOffset );
	b3Vec3 normal = joint->normal;

	// Track the contact separation through the substeps
	b3Vec3 dA = b3Add( stateA->deltaPosition, b3Sub( rA, joint->anchorA ) );
	b3Vec3 dB = b3Add( stateB->deltaPosition, b3Sub( rB, joint->anchorB ) );
	float deltaC = b3Dot( normal, b3Sub( dA, dB ) );

	// Suspension spring, push only. Softness is relative to this row's effective mass so the
	// spring behaves as the requested stiffness and damping in N/m and N*s/m.
	if ( joint->stiffness > 0.0f && joint->normalMass > 0.0f )
	{
		b3Vec3 vr = b3Sub( b3Add( vA, b3Cross( wA, rA ) ), b3Add( vB, b3Cross( wB, rB ) ) );
		float cdot = b3Dot( normal, vr );
		float C = joint->springC + deltaC;

		float damping = cdot < 0.0f ? joint->compressionDamping : joint->reboundDamping;
		float effMass = joint->normalMass;
		float omega = sqrtf( joint->stiffness / effMass );
		float zeta = damping / ( 2.0f * effMass * omega );
		b3Softness soft = b3MakeSoftOmega( omega, zeta, h );

		float impulse = -effMass * soft.massScale * ( cdot + soft.biasRate * C ) - soft.impulseScale * joint->springImpulse;
		float newImpulse = b3MaxFloat( joint->springImpulse + impulse, 0.0f );
		impulse = newImpulse - joint->springImpulse;
		joint->springImpulse = newImpulse;

		b3ApplyWheelImpulse( joint, mA, mB, iA, iB, rA, rB, b3MulSV( impulse, normal ), &vA, &wA, &vB, &wB );
	}

	// Hard bump stop, push only, speculative with no position correction
	if ( joint->enableBumpStop )
	{
		b3Vec3 vr = b3Sub( b3Add( vA, b3Cross( wA, rA ) ), b3Add( vB, b3Cross( wB, rB ) ) );
		float cdot = b3Dot( normal, vr );
		float C = joint->bumpC + deltaC;
		float bias = C > 0.0f ? C * context->inv_h : 0.0f;
		B3_UNUSED( useBias );

		float impulse = -joint->normalMass * ( cdot + bias );
		float newImpulse = b3MaxFloat( joint->bumpImpulse + impulse, 0.0f );
		impulse = newImpulse - joint->bumpImpulse;
		joint->bumpImpulse = newImpulse;

		b3ApplyWheelImpulse( joint, mA, mB, iA, iB, rA, rB, b3MulSV( impulse, normal ), &vA, &wA, &vB, &wB );
	}

	float normalImpulse = joint->springImpulse + joint->bumpImpulse;
	// The wheel spin is too light for a force limit that lags the slip by a step, so the roller
	// holds up to the peak grip and only falls to the sliding grip once the tire is sliding.
	float maxLongitudinal = joint->longitudinalGrip * normalImpulse;
	float maxLateral = joint->lateralFriction * normalImpulse;

	// Longitudinal roller: drives the contact patch slip (v - omega * R) to zero by trading
	// momentum between the chassis/ground and the wheel spin.
	bool locked = false;

	// The wheel spin is much lighter than the chassis, so the brake and the roller are solved as one
	// small mixed problem: first try holding the wheel still with the brake, and if the brake cannot
	// hold it, saturate the brake and let the wheel roll.
	{
		b3Vec3 vr = b3Sub( b3Add( vA, b3Cross( wA, rA ) ), b3Add( vB, b3Cross( wB, rB ) ) );
		float vLong = b3Dot( joint->forward, vr );
		float R = joint->radius;
		float invI = joint->invInertia;
		float oldImpulse = joint->longitudinalImpulse;
		float maxBrake = h * joint->brakeTorque;

		if ( maxBrake > 0.0f && invI > 0.0f && joint->lockedMass > 0.0f )
		{
			float lockedImpulse =
				b3ClampFloat( oldImpulse - joint->lockedMass * vLong, -maxLongitudinal, maxLongitudinal );
			float impulse = lockedImpulse - oldImpulse;
			float brakeImpulse = joint->brakeImpulse + R * impulse - joint->omega / invI;
			if ( -maxBrake <= brakeImpulse && brakeImpulse <= maxBrake )
			{
				locked = true;
				joint->brakeImpulse = brakeImpulse;
				joint->longitudinalImpulse = lockedImpulse;
				joint->omega = 0.0f;
				b3ApplyWheelImpulse( joint, mA, mB, iA, iB, rA, rB, b3MulSV( impulse, joint->forward ), &vA, &wA, &vB,
									 &wB );
			}
			else
			{
				float newBrake = brakeImpulse > 0.0f ? maxBrake : -maxBrake;
				float delta = newBrake - joint->brakeImpulse;

				// The brake cannot reverse the spin
				if ( delta * joint->omega < 0.0f && b3AbsFloat( delta * invI ) > b3AbsFloat( joint->omega ) )
				{
					delta = -joint->omega / invI;
					newBrake = joint->brakeImpulse + delta;
				}

				joint->brakeImpulse = newBrake;
				joint->omega += invI * delta;
			}
		}

		if ( locked == false )
		{
			float slip = vLong - joint->omega * R;
			float impulse = -joint->longitudinalMass * slip;
			joint->longitudinalImpulse = b3ClampFloat( oldImpulse + impulse, -maxLongitudinal, maxLongitudinal );
			impulse = joint->longitudinalImpulse - oldImpulse;

			b3ApplyWheelImpulse( joint, mA, mB, iA, iB, rA, rB, b3MulSV( impulse, joint->forward ), &vA, &wA, &vB,
								 &wB );
			joint->omega -= R * invI * impulse;
		}
	}

	// Lateral row, applied at the raised point on the chassis
	{
		b3Vec3 vr = b3Sub( b3Add( vA, b3Cross( wA, rLatA ) ), b3Add( vB, b3Cross( wB, rB ) ) );
		float cdot = b3Dot( joint->lateral, vr );
		float impulse = -joint->lateralMass * cdot;
		float oldImpulse = joint->lateralImpulse;
		joint->lateralImpulse = b3ClampFloat( oldImpulse + impulse, -maxLateral, maxLateral );
		impulse = joint->lateralImpulse - oldImpulse;

		b3ApplyWheelImpulse( joint, mA, mB, iA, iB, rLatA, rB, b3MulSV( impulse, joint->lateral ), &vA, &wA, &vB,
							 &wB );
	}

	// Friction ellipse: scale both impulses back onto the ellipse when combined slip exceeds it
	float gripLongitudinal = joint->longitudinalGrip * normalImpulse;
	float gripLateral = joint->lateralGrip * normalImpulse;
	if ( gripLongitudinal > 0.0f && gripLateral > 0.0f )
	{
		float x = joint->longitudinalImpulse / gripLongitudinal;
		float y = joint->lateralImpulse / gripLateral;
		float e = x * x + y * y;
		if ( e > 1.0f )
		{
			float scale = 1.0f / sqrtf( e );
			float newLong = scale * joint->longitudinalImpulse;
			float newLat = scale * joint->lateralImpulse;
			float dLong = newLong - joint->longitudinalImpulse;
			float dLat = newLat - joint->lateralImpulse;
			joint->longitudinalImpulse = newLong;
			joint->lateralImpulse = newLat;

			b3ApplyWheelImpulse( joint, mA, mB, iA, iB, rA, rB, b3MulSV( dLong, joint->forward ), &vA, &wA, &vB,
								 &wB );
			if ( locked )
			{
				joint->brakeImpulse += joint->radius * dLong;
			}
			else
			{
				joint->omega -= joint->radius * joint->invInertia * dLong;
			}
			b3ApplyWheelImpulse( joint, mA, mB, iA, iB, rLatA, rB, b3MulSV( dLat, joint->lateral ), &vA, &wA, &vB,
								 &wB );
		}
	}

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

void b3DrawVehicleWheelJoint( b3DebugDraw* draw, b3World* world, b3JointSim* base, b3WorldTransform transformA,
							  b3WorldTransform transformB )
{
	B3_ASSERT( base->type == b3_vehicleWheelJoint );
	B3_UNUSED( transformA );

	b3VehicleWheelJoint* joint = &base->vehicleWheelJoint;
	if ( joint->hasContact == false )
	{
		return;
	}

	b3Pos p = b3TransformWorldPoint( transformB, base->localFrameB.p );
	float normalForce = world->inv_h * ( joint->springImpulse + joint->bumpImpulse );
	float longForce = world->inv_h * joint->longitudinalImpulse;
	float latForce = world->inv_h * joint->lateralImpulse;

	// 1 m per 10 kN
	float scale = 1.0e-4f * b3GetLengthUnitsPerMeter();
	draw->DrawPointFcn( p, 6.0f, joint->bumpImpulse > 0.0f ? b3_colorRed : b3_colorLimeGreen, draw->context );
	draw->DrawSegmentFcn( p, b3OffsetPos( p, b3MulSV( scale * normalForce, joint->normal ) ), b3_colorLimeGreen,
						  draw->context );
	draw->DrawSegmentFcn( p, b3OffsetPos( p, b3MulSV( scale * longForce, joint->forward ) ), b3_colorOrange,
						  draw->context );
	draw->DrawSegmentFcn( p, b3OffsetPos( p, b3MulSV( scale * latForce, joint->lateral ) ), b3_colorDeepSkyBlue,
						  draw->context );

	if ( draw->drawJointExtras )
	{
		char buffer[96];
		snprintf( buffer, 96, "n = %.0f, x = %.0f, y = %.0f, w = %.1f", normalForce, longForce, latForce,
				  joint->omega );
		draw->DrawStringFcn( p, buffer, b3_colorWhite, draw->context );
	}
}

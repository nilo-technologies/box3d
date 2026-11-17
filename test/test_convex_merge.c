// SPDX-FileCopyrightText: 2026 Nilo Technologies
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// A wall 4 wide, 3 tall and 0.3 thick with a doorway 1 wide and 2.1 tall cut into the bottom.
// The outline runs counter-clockwise seen from +z.
#define OUTLINE_COUNT 8
static const float s_outline[OUTLINE_COUNT][2] = {
	{ -2.0f, 0.0f }, { -0.5f, 0.0f }, { -0.5f, 2.1f }, { 0.5f, 2.1f },
	{ 0.5f, 0.0f },	 { 2.0f, 0.0f },  { 2.0f, 3.0f },  { -2.0f, 3.0f },
};

#define CAP_TRIANGLE_COUNT 6
static const int s_capTriangles[CAP_TRIANGLE_COUNT][3] = {
	{ 0, 1, 2 }, { 0, 2, 7 }, { 2, 3, 7 }, { 3, 6, 7 }, { 3, 4, 5 }, { 3, 5, 6 },
};

#define WALL_VERTEX_COUNT ( 2 * OUTLINE_COUNT )
#define WALL_TRIANGLE_COUNT ( 2 * CAP_TRIANGLE_COUNT + 2 * OUTLINE_COUNT )
#define WALL_HALF_THICKNESS 0.15f

typedef struct WallMesh
{
	b3Vec3 vertices[WALL_VERTEX_COUNT];
	int32_t indices[3 * WALL_TRIANGLE_COUNT];
} WallMesh;

// Front vertices first, then back. Reversing the winding turns the mesh inside out.
static void MakeWall( WallMesh* wall, bool clockwise )
{
	for ( int i = 0; i < OUTLINE_COUNT; ++i )
	{
		wall->vertices[i] = ( b3Vec3 ){ s_outline[i][0], s_outline[i][1], WALL_HALF_THICKNESS };
		wall->vertices[OUTLINE_COUNT + i] =
			( b3Vec3 ){ s_outline[i][0], s_outline[i][1], -WALL_HALF_THICKNESS };
	}

	int count = 0;
	for ( int i = 0; i < CAP_TRIANGLE_COUNT; ++i )
	{
		const int* triangle = s_capTriangles[i];
		wall->indices[count++] = triangle[0];
		wall->indices[count++] = triangle[1];
		wall->indices[count++] = triangle[2];
		wall->indices[count++] = OUTLINE_COUNT + triangle[0];
		wall->indices[count++] = OUTLINE_COUNT + triangle[2];
		wall->indices[count++] = OUTLINE_COUNT + triangle[1];
	}

	for ( int i = 0; i < OUTLINE_COUNT; ++i )
	{
		int j = ( i + 1 ) % OUTLINE_COUNT;
		wall->indices[count++] = i;
		wall->indices[count++] = OUTLINE_COUNT + i;
		wall->indices[count++] = OUTLINE_COUNT + j;
		wall->indices[count++] = i;
		wall->indices[count++] = OUTLINE_COUNT + j;
		wall->indices[count++] = j;
	}

	if ( clockwise )
	{
		for ( int i = 0; i < count; i += 3 )
		{
			int32_t swap = wall->indices[i + 1];
			wall->indices[i + 1] = wall->indices[i + 2];
			wall->indices[i + 2] = swap;
		}
	}
}

// The wall cut into six boxes: each side of the doorway in two stacked halves, and the lintel in
// a left and a right half. Three hulls cover it without sealing anything.
#define PIECE_COUNT 6
static const float s_pieceBoxes[PIECE_COUNT][4] = {
	// x0, y0, x1, y1
	{ -2.0f, 0.0f, -0.5f, 1.5f }, { -2.0f, 1.5f, -0.5f, 3.0f }, { 0.5f, 0.0f, 2.0f, 1.5f },
	{ 0.5f, 1.5f, 2.0f, 3.0f },	  { -0.5f, 2.1f, 0.0f, 3.0f },	{ 0.0f, 2.1f, 0.5f, 3.0f },
};

typedef struct WallPieces
{
	b3Vec3 points[8 * PIECE_COUNT];
	int32_t offsets[PIECE_COUNT + 1];
} WallPieces;

static void MakePieces( WallPieces* pieces )
{
	for ( int i = 0; i < PIECE_COUNT; ++i )
	{
		const float* box = s_pieceBoxes[i];
		pieces->offsets[i] = 8 * i;
		for ( int j = 0; j < 8; ++j )
		{
			float x = ( j & 1 ) ? box[2] : box[0];
			float y = ( j & 2 ) ? box[3] : box[1];
			float z = ( j & 4 ) ? WALL_HALF_THICKNESS : -WALL_HALF_THICKNESS;
			pieces->points[8 * i + j] = ( b3Vec3 ){ x, y, z };
		}
	}

	pieces->offsets[PIECE_COUNT] = 8 * PIECE_COUNT;
}

// The pieces of the wall, merged with the defaults otherwise: the limit is found by a sweep
static b3ConvexMergeDef MakeSweepDef( const WallMesh* wall, const WallPieces* pieces )
{
	b3ConvexMergeDef def = b3DefaultConvexMergeDef();
	def.seedTriangles = false;
	def.meshVertices = wall->vertices;
	def.meshIndices = wall->indices;
	def.meshVertexCount = WALL_VERTEX_COUNT;
	def.meshTriangleCount = WALL_TRIANGLE_COUNT;
	def.piecePoints = pieces->points;
	def.pieceOffsets = pieces->offsets;
	def.pieceCount = PIECE_COUNT;
	return def;
}

// The wall merged once, to a limit of 5 cm
static b3ConvexMergeDef MakeWallDef( const WallMesh* wall, const WallPieces* pieces )
{
	b3ConvexMergeDef def = MakeSweepDef( wall, pieces );
	def.enableSweep = false;
	def.maxConcavity = 0.05f;
	return def;
}

static bool IsSameResult( const b3ConvexMergeResult* a, const b3ConvexMergeResult* b )
{
	if ( a->pieceCount != b->pieceCount || a->maxConcavity != b->maxConcavity )
	{
		return false;
	}

	for ( int i = 0; i <= a->pieceCount; ++i )
	{
		if ( a->offsets[i] != b->offsets[i] )
		{
			return false;
		}
	}

	for ( int i = 0; i < a->pieceCount; ++i )
	{
		if ( a->concavities[i] != b->concavities[i] )
		{
			return false;
		}
	}

	for ( int i = 0; i < a->offsets[a->pieceCount]; ++i )
	{
		if ( a->points[i].x != b->points[i].x || a->points[i].y != b->points[i].y ||
			 a->points[i].z != b->points[i].z )
		{
			return false;
		}
	}

	return true;
}

// Is the point inside one of the merged hulls?
static bool IsCovered( const b3ConvexMergeResult* result, b3Vec3 point )
{
	for ( int i = 0; i < result->pieceCount; ++i )
	{
		int count = result->offsets[i + 1] - result->offsets[i];
		b3HullData* hull = b3CreateHull( result->points + result->offsets[i], count, count );
		if ( hull == NULL )
		{
			continue;
		}

		const b3Plane* planes = b3GetHullPlanes( hull );
		bool inside = true;
		for ( int j = 0; j < hull->faceCount; ++j )
		{
			if ( b3Dot( planes[j].normal, point ) - planes[j].offset > 0.0f )
			{
				inside = false;
				break;
			}
		}

		b3DestroyHull( hull );
		if ( inside )
		{
			return true;
		}
	}

	return false;
}

static float GetVolume( const b3ConvexMergeResult* result )
{
	float volume = 0.0f;
	for ( int i = 0; i < result->pieceCount; ++i )
	{
		int count = result->offsets[i + 1] - result->offsets[i];
		b3HullData* hull = b3CreateHull( result->points + result->offsets[i], count, count );
		if ( hull != NULL )
		{
			volume += hull->volume;
			b3DestroyHull( hull );
		}
	}

	return volume;
}

// The doorway stays open: the six boxes become the three hulls that seal nothing.
static int MergeKeepsDoorwayOpen( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );
	ENSURE( result->offsets[0] == 0 );
	ENSURE( result->offsets[3] == 24 );
	ENSURE_SMALL( result->maxConcavity, 1e-3f );

	// The wall is covered and the doorway is not
	ENSURE( IsCovered( result, ( b3Vec3 ){ -1.2f, 1.0f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 1.2f, 2.5f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 2.6f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.4f, 2.0f, 0.0f } ) == false );

	// Wall volume: 4 x 3 x 0.3 less the doorway 1 x 2.1 x 0.3
	ENSURE_SMALL( GetVolume( result ) - 2.97f, 1e-3f );

	// Without a sweep the one limit is the one step
	ENSURE( result->stepCount == 1 );
	ENSURE( result->bestStep == 0 );
	ENSURE( result->steps[0].maxConcavity == 0.05f );
	ENSURE( result->steps[0].hullCount == 3 );
	ENSURE( result->steps[0].sealedDepth == result->maxConcavity );

	b3DestroyConvexMergeResult( result );
	return 0;
}

// A zero limit still merges everything that seals nothing.
static int MergeWithZeroConcavity( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.maxConcavity = 0.0f;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );

	b3DestroyConvexMergeResult( result );
	return 0;
}

// The outward side comes from the signed volume, so a mesh wound the other way merges the same.
static int MergeIgnoresWinding( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, true );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );

	b3DestroyConvexMergeResult( result );
	return 0;
}

// A hull budget forces merges past the limit, and the result says what that sealed.
static int MergeMeetsHullBudget( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.maxHullCount = 2;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 2 );
	ENSURE( result->maxConcavity > 0.25f );
	b3DestroyConvexMergeResult( result );

	// One hull is the whole wall with the doorway sealed, 1 wide. The depth is read off a
	// ladder of distances, so it is an estimate.
	def.maxHullCount = 1;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 1 );
	ENSURE( result->maxConcavity > 0.9f );
	ENSURE( result->maxConcavity < 1.2f );
	ENSURE( result->concavities[0] == result->maxConcavity );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) );
	ENSURE_SMALL( GetVolume( result ) - 3.6f, 1e-3f );
	b3DestroyConvexMergeResult( result );

	// A budget the limit already meets changes nothing
	def.maxHullCount = 5;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );
	b3DestroyConvexMergeResult( result );
	return 0;
}

// A limit as wide as the doorway lets the whole wall become one hull.
static int MergeWithLooseConcavity( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.maxConcavity = 1.5f;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 1 );
	ENSURE( result->maxConcavity <= 1.5f );

	b3DestroyConvexMergeResult( result );
	return 0;
}

// A piece too small to be a hull is dropped, the rest merge as before.
static int MergeDropsDegeneratePiece( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	// Six boxes, then three points and four coplanar points
	b3Vec3 points[8 * PIECE_COUNT + 7];
	int32_t offsets[PIECE_COUNT + 3];
	for ( int i = 0; i < 8 * PIECE_COUNT; ++i )
	{
		points[i] = pieces.points[i];
	}

	for ( int i = 0; i <= PIECE_COUNT; ++i )
	{
		offsets[i] = pieces.offsets[i];
	}

	points[48] = ( b3Vec3 ){ -1.0f, 1.0f, 0.0f };
	points[49] = ( b3Vec3 ){ -1.1f, 1.0f, 0.0f };
	points[50] = ( b3Vec3 ){ -1.0f, 1.1f, 0.0f };
	offsets[PIECE_COUNT + 1] = 51;
	points[51] = ( b3Vec3 ){ 1.0f, 1.0f, 0.0f };
	points[52] = ( b3Vec3 ){ 1.1f, 1.0f, 0.0f };
	points[53] = ( b3Vec3 ){ 1.1f, 1.1f, 0.0f };
	points[54] = ( b3Vec3 ){ 1.0f, 1.1f, 0.0f };
	offsets[PIECE_COUNT + 2] = 55;

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.piecePoints = points;
	def.pieceOffsets = offsets;
	def.pieceCount = PIECE_COUNT + 2;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );

	b3DestroyConvexMergeResult( result );
	return 0;
}

// The vertex limit holds for every merged hull. Six is under the eight corners of each box and
// of the wall they merge into, so both the pieces and the merged hull have to be simplified.
static int MergeRespectsVertexLimit( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.maxHullVertices = 6;
	def.maxHullCount = 1;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 1 );
	ENSURE( result->offsets[1] - result->offsets[0] <= 6 );
	ENSURE( result->offsets[1] - result->offsets[0] >= 4 );
	b3DestroyConvexMergeResult( result );

	// Without a budget the limit applies to whatever hulls remain
	def.maxHullCount = 0;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount >= 1 );
	for ( int i = 0; i < result->pieceCount; ++i )
	{
		ENSURE( result->offsets[i + 1] - result->offsets[i] <= 6 );
		ENSURE( result->offsets[i + 1] - result->offsets[i] >= 4 );
	}

	b3DestroyConvexMergeResult( result );
	return 0;
}

static int MergeRejectsBadInput( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );

	ENSURE( b3MergeConvexPieces( NULL ) == NULL );

	b3ConvexMergeDef bad = def;
	bad.meshVertices = NULL;
	ENSURE( b3MergeConvexPieces( &bad ) == NULL );

	bad = def;
	bad.meshTriangleCount = 0;
	ENSURE( b3MergeConvexPieces( &bad ) == NULL );

	bad = def;
	bad.piecePoints = NULL;
	ENSURE( b3MergeConvexPieces( &bad ) == NULL );

	bad = def;
	bad.pieceCount = 0;
	ENSURE( b3MergeConvexPieces( &bad ) == NULL );

	// A triangle index past the vertices
	WallMesh broken = wall;
	broken.indices[4] = WALL_VERTEX_COUNT;
	bad = def;
	bad.meshIndices = broken.indices;
	ENSURE( b3MergeConvexPieces( &bad ) == NULL );

	// Offsets that run backwards
	WallPieces shuffled = pieces;
	shuffled.offsets[2] = 4;
	bad = def;
	bad.pieceOffsets = shuffled.offsets;
	ENSURE( b3MergeConvexPieces( &bad ) == NULL );

	// Destroying nothing is fine
	b3DestroyConvexMergeResult( NULL );
	return 0;
}

// Merging the same input twice gives the same hulls.
static int MergeIsRepeatable( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.maxHullCount = 2;
	b3ConvexMergeResult* first = b3MergeConvexPieces( &def );
	b3ConvexMergeResult* second = b3MergeConvexPieces( &def );
	ENSURE( first != NULL && second != NULL );
	ENSURE( IsSameResult( first, second ) );

	b3DestroyConvexMergeResult( first );
	b3DestroyConvexMergeResult( second );
	return 0;
}

// The default is a sweep: nine limits spread over a range that follows the size of the mesh, the
// hull count never rising along them, and one step chosen.
static int SweepIsTheDefault( void )
{
	b3ConvexMergeDef defaults = b3DefaultConvexMergeDef();
	ENSURE( defaults.enableSweep );
	ENSURE( defaults.seedTriangles );
	ENSURE( defaults.enableSplit == false );
	ENSURE( defaults.sweepSteps == 9 );
	ENSURE( defaults.hullWeight == 1.0f );
	ENSURE( defaults.minConcavity == 0.0f && defaults.maxConcavity == 0.0f );

	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeSweepDef( &wall, &pieces );
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 9 );

	// 0.5% to 8% of the diagonal of the wall, each limit 1.41 times the one before
	float diagonal = sqrtf( 4.0f * 4.0f + 3.0f * 3.0f + 0.3f * 0.3f );
	ENSURE_SMALL( result->steps[0].maxConcavity - 0.005f * diagonal, 1e-5f );
	ENSURE_SMALL( result->steps[8].maxConcavity - 0.08f * diagonal, 1e-5f );
	for ( int i = 1; i < result->stepCount; ++i )
	{
		ENSURE_SMALL( result->steps[i].maxConcavity / result->steps[i - 1].maxConcavity -
						  sqrtf( 2.0f ),
					  1e-3f );
		ENSURE( result->steps[i].hullCount <= result->steps[i - 1].hullCount );
		ENSURE( result->steps[i].sealedDepth >= result->steps[i - 1].sealedDepth );
	}

	// The hulls are those of the chosen step, the first at its hull count
	ENSURE( result->bestStep >= 0 && result->bestStep < result->stepCount );
	ENSURE( result->pieceCount == result->steps[result->bestStep].hullCount );
	ENSURE( result->maxConcavity == result->steps[result->bestStep].sealedDepth );
	ENSURE( result->bestStep == 0 || result->steps[result->bestStep].hullCount <
										 result->steps[result->bestStep - 1].hullCount );

	// The doorway is 1 wide and the sweep stops at 40 cm: it stays open
	ENSURE( result->pieceCount == 3 );
	ENSURE( IsCovered( result, ( b3Vec3 ){ -1.2f, 1.0f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );

	b3DestroyConvexMergeResult( result );
	return 0;
}

// A sweep of one step, and the first step of any sweep, are the merge at that limit alone, hull
// for hull.
static int SweepStartsAsOneMerge( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	b3ConvexMergeResult* single = b3MergeConvexPieces( &def );
	ENSURE( single != NULL );

	b3ConvexMergeDef sweep = MakeSweepDef( &wall, &pieces );
	sweep.maxConcavity = 0.05f;
	sweep.sweepSteps = 1;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &sweep );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 1 );
	ENSURE( result->steps[0].maxConcavity == 0.05f );
	ENSURE( IsSameResult( single, result ) );
	b3DestroyConvexMergeResult( result );

	// A weight that only cares for the fit keeps the first step
	sweep.minConcavity = 0.05f;
	sweep.maxConcavity = 1.5f;
	sweep.sweepSteps = 4;
	sweep.hullWeight = 0.001f;
	result = b3MergeConvexPieces( &sweep );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 4 );
	ENSURE( result->bestStep == 0 );
	ENSURE( result->steps[0].maxConcavity == 0.05f );
	ENSURE( IsSameResult( single, result ) );
	b3DestroyConvexMergeResult( result );

	b3DestroyConvexMergeResult( single );
	return 0;
}

// The range and the weight are the definition's to give. A limit as wide as the doorway lets the
// last step close it, and the weight decides whether that step is the one returned.
static int SweepFollowsRangeAndWeight( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeSweepDef( &wall, &pieces );
	def.minConcavity = 0.5f;
	def.maxConcavity = 1.5f;
	def.sweepSteps = 3;

	// Fewer hulls at any cost: the one hull that seals the doorway
	def.hullWeight = 100.0f;
	b3ConvexMergeResult* fewer = b3MergeConvexPieces( &def );
	ENSURE( fewer != NULL );
	ENSURE( fewer->stepCount == 3 );
	ENSURE( fewer->steps[0].maxConcavity == 0.5f );
	ENSURE_SMALL( fewer->steps[1].maxConcavity - sqrtf( 0.5f * 1.5f ), 1e-5f );
	ENSURE( fewer->steps[2].maxConcavity == 1.5f );
	ENSURE( fewer->steps[2].hullCount == 1 );
	ENSURE( fewer->pieceCount == 1 );
	ENSURE( IsCovered( fewer, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) );

	// A closer fit at any cost: the first step, with the doorway open
	def.hullWeight = 0.001f;
	b3ConvexMergeResult* closer = b3MergeConvexPieces( &def );
	ENSURE( closer != NULL );
	ENSURE( closer->bestStep == 0 );
	ENSURE( closer->pieceCount > 1 );
	ENSURE( IsCovered( closer, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );

	// The steps are the same whatever the weight
	for ( int i = 0; i < 3; ++i )
	{
		ENSURE( fewer->steps[i].hullCount == closer->steps[i].hullCount );
		ENSURE( fewer->steps[i].sealedDepth == closer->steps[i].sealedDepth );
	}

	// A weight that is no weight counts as 1
	def.hullWeight = 1.0f;
	b3ConvexMergeResult* even = b3MergeConvexPieces( &def );
	def.hullWeight = -2.0f;
	b3ConvexMergeResult* negative = b3MergeConvexPieces( &def );
	def.hullWeight = NAN;
	b3ConvexMergeResult* notNumber = b3MergeConvexPieces( &def );
	ENSURE( even != NULL && negative != NULL && notNumber != NULL );
	ENSURE( negative->bestStep == even->bestStep );
	ENSURE( notNumber->bestStep == even->bestStep );

	b3DestroyConvexMergeResult( fewer );
	b3DestroyConvexMergeResult( closer );
	b3DestroyConvexMergeResult( even );
	b3DestroyConvexMergeResult( negative );
	b3DestroyConvexMergeResult( notNumber );
	return 0;
}

// maxConcavity caps a sweep, and the step count is clamped.
static int SweepStaysInBounds( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	// A cap under the tightest limit the sweep would pick itself (2.5 cm here) leaves one step
	b3ConvexMergeDef def = MakeSweepDef( &wall, &pieces );
	def.maxConcavity = 0.01f;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 1 );
	ENSURE( result->steps[0].maxConcavity == 0.01f );
	b3DestroyConvexMergeResult( result );

	// So does a range given upside down
	def.minConcavity = 0.3f;
	def.maxConcavity = 0.1f;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 1 );
	ENSURE( result->steps[0].maxConcavity == 0.1f );
	b3DestroyConvexMergeResult( result );

	// A range that is not a number falls back to the size of the mesh
	def.minConcavity = NAN;
	def.maxConcavity = -1.0f;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 9 );
	ENSURE( result->steps[0].maxConcavity > 0.02f && result->steps[8].maxConcavity < 0.41f );
	b3DestroyConvexMergeResult( result );

	def = MakeSweepDef( &wall, &pieces );
	def.sweepSteps = 0;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 1 );
	b3DestroyConvexMergeResult( result );

	def.sweepSteps = -5;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 1 );
	b3DestroyConvexMergeResult( result );

	def.sweepSteps = 1000;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 64 );
	for ( int i = 1; i < result->stepCount; ++i )
	{
		ENSURE( result->steps[i].maxConcavity > result->steps[i - 1].maxConcavity );
		ENSURE( result->steps[i].hullCount <= result->steps[i - 1].hullCount );
	}

	b3DestroyConvexMergeResult( result );
	return 0;
}

// A hull budget holds at every step of a sweep.
static int SweepMeetsHullBudget( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeSweepDef( &wall, &pieces );
	def.maxHullCount = 2;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 9 );
	for ( int i = 0; i < result->stepCount; ++i )
	{
		ENSURE( result->steps[i].hullCount <= 2 );
	}

	ENSURE( result->pieceCount <= 2 );
	ENSURE( result->maxConcavity > 0.25f );
	b3DestroyConvexMergeResult( result );
	return 0;
}

// A sweep checks the definition as a single merge does, and gives the same hulls twice.
static int SweepIsRepeatable( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeSweepDef( &wall, &pieces );
	def.maxConcavity = 1.5f;
	b3ConvexMergeResult* first = b3MergeConvexPieces( &def );
	b3ConvexMergeResult* second = b3MergeConvexPieces( &def );
	ENSURE( first != NULL && second != NULL );
	ENSURE( first->stepCount == second->stepCount );
	ENSURE( first->bestStep == second->bestStep );
	ENSURE( IsSameResult( first, second ) );
	b3DestroyConvexMergeResult( first );
	b3DestroyConvexMergeResult( second );

	b3ConvexMergeDef bad = def;
	bad.piecePoints = NULL;
	ENSURE( b3MergeConvexPieces( &bad ) == NULL );
	return 0;
}

// One piece over the whole wall seals the doorway. Fitted to the mesh and cut where it seals,
// it comes apart into pieces that leave the doorway open.
static int SplitOpensDoorway( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	// A box a little larger than the wall, as a voxel piece would be
	b3Vec3 box[8];
	for ( int j = 0; j < 8; ++j )
	{
		box[j] = ( b3Vec3 ){ ( j & 1 ) ? 2.05f : -2.05f, ( j & 2 ) ? 3.05f : -0.05f,
							 ( j & 4 ) ? 0.2f : -0.2f };
	}

	int32_t offsets[2] = { 0, 8 };
	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.piecePoints = box;
	def.pieceOffsets = offsets;
	def.pieceCount = 1;

	// As it comes, the piece is the result
	b3ConvexMergeResult* sealed = b3MergeConvexPieces( &def );
	ENSURE( sealed != NULL );
	ENSURE( sealed->pieceCount == 1 );
	ENSURE( sealed->maxConcavity > 0.9f );
	ENSURE( IsCovered( sealed, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) );
	b3DestroyConvexMergeResult( sealed );

	def.enableSplit = true;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );
	ENSURE_SMALL( result->maxConcavity, 1e-3f );
	ENSURE( IsCovered( result, ( b3Vec3 ){ -1.2f, 1.0f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 1.2f, 2.5f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 2.6f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );

	// The pieces were fitted to the wall: nothing is left of the margin of the box. Where two
	// pieces meet they stop a hair short of each other.
	ENSURE_SMALL( GetVolume( result ) - 2.97f, 5e-3f );
	b3DestroyConvexMergeResult( result );

	// Pieces that seal nothing are fitted and left whole. Either winding works.
	MakeWall( &wall, true );
	def = MakeWallDef( &wall, &pieces );
	def.enableSplit = true;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );
	ENSURE_SMALL( GetVolume( result ) - 2.97f, 5e-3f );
	b3DestroyConvexMergeResult( result );
	return 0;
}

// Seeded with the triangles of the mesh the merge needs no pieces: the faces of the wall grow
// into the hulls around the doorway.
static int SeedTrianglesKeepsDoorwayOpen( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.piecePoints = NULL;
	def.pieceOffsets = NULL;
	def.pieceCount = 0;
	ENSURE( b3MergeConvexPieces( &def ) == NULL );

	def.seedTriangles = true;
	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );
	ENSURE_SMALL( result->maxConcavity, 1e-3f );
	ENSURE( IsCovered( result, ( b3Vec3 ){ -1.2f, 1.0f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 1.2f, 2.5f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 2.6f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );
	ENSURE_SMALL( GetVolume( result ) - 2.97f, 1e-3f );
	b3DestroyConvexMergeResult( result );

	// The same with the mesh wound the other way, and with a limit as wide as the doorway
	MakeWall( &wall, true );
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 3 );
	b3DestroyConvexMergeResult( result );

	def.maxConcavity = 1.5f;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 1 );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) );
	b3DestroyConvexMergeResult( result );

	// A hull budget and the sweep work as they do on pieces
	def.maxConcavity = 0.05f;
	def.maxHullCount = 2;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->pieceCount == 2 );
	b3DestroyConvexMergeResult( result );

	def = MakeSweepDef( &wall, &pieces );
	def.seedTriangles = true;
	result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 9 );
	ENSURE( result->pieceCount == 3 );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );
	b3DestroyConvexMergeResult( result );
	return 0;
}

// The default needs nothing but the mesh: the merge starts from its triangles and finds its limit.
static int DefaultNeedsOnlyTheMesh( void )
{
	WallMesh wall;
	MakeWall( &wall, false );

	b3ConvexMergeDef def = b3DefaultConvexMergeDef();
	def.meshVertices = wall.vertices;
	def.meshIndices = wall.indices;
	def.meshVertexCount = WALL_VERTEX_COUNT;
	def.meshTriangleCount = WALL_TRIANGLE_COUNT;

	b3ConvexMergeResult* result = b3MergeConvexPieces( &def );
	ENSURE( result != NULL );
	ENSURE( result->stepCount == 9 );
	ENSURE( result->pieceCount == 3 );
	ENSURE_SMALL( result->maxConcavity, 1e-3f );
	ENSURE( IsCovered( result, ( b3Vec3 ){ -1.2f, 1.0f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 2.6f, 0.0f } ) );
	ENSURE( IsCovered( result, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );
	ENSURE_SMALL( GetVolume( result ) - 2.97f, 1e-3f );

	// Pieces given with it are not used
	WallPieces pieces;
	MakePieces( &pieces );
	b3Vec3 box[8];
	for ( int j = 0; j < 8; ++j )
	{
		box[j] = ( b3Vec3 ){ ( j & 1 ) ? 2.0f : -2.0f, ( j & 2 ) ? 3.0f : 0.0f,
							 ( j & 4 ) ? 0.15f : -0.15f };
	}

	int32_t offsets[2] = { 0, 8 };
	def.piecePoints = box;
	def.pieceOffsets = offsets;
	def.pieceCount = 1;
	b3ConvexMergeResult* again = b3MergeConvexPieces( &def );
	ENSURE( again != NULL );
	ENSURE( IsSameResult( result, again ) );

	// The same twice
	b3ConvexMergeResult* twice = b3MergeConvexPieces( &def );
	ENSURE( twice != NULL );
	ENSURE( IsSameResult( result, twice ) );

	b3DestroyConvexMergeResult( result );
	b3DestroyConvexMergeResult( again );
	b3DestroyConvexMergeResult( twice );

	// A mesh is still needed
	def.meshIndices = NULL;
	ENSURE( b3MergeConvexPieces( &def ) == NULL );
	return 0;
}

int ConvexMergeTest( void )
{
	RUN_SUBTEST( MergeKeepsDoorwayOpen );
	RUN_SUBTEST( MergeWithZeroConcavity );
	RUN_SUBTEST( MergeIgnoresWinding );
	RUN_SUBTEST( MergeMeetsHullBudget );
	RUN_SUBTEST( MergeWithLooseConcavity );
	RUN_SUBTEST( MergeDropsDegeneratePiece );
	RUN_SUBTEST( MergeRespectsVertexLimit );
	RUN_SUBTEST( MergeRejectsBadInput );
	RUN_SUBTEST( MergeIsRepeatable );
	RUN_SUBTEST( SweepIsTheDefault );
	RUN_SUBTEST( SweepStartsAsOneMerge );
	RUN_SUBTEST( SweepFollowsRangeAndWeight );
	RUN_SUBTEST( SweepStaysInBounds );
	RUN_SUBTEST( SweepMeetsHullBudget );
	RUN_SUBTEST( SweepIsRepeatable );
	RUN_SUBTEST( SplitOpensDoorway );
	RUN_SUBTEST( SeedTrianglesKeepsDoorwayOpen );
	RUN_SUBTEST( DefaultNeedsOnlyTheMesh );

	return 0;
}

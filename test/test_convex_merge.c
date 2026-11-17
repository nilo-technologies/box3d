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

static b3ConvexMergeDef MakeWallDef( const WallMesh* wall, const WallPieces* pieces )
{
	b3ConvexMergeDef def = b3DefaultConvexMergeDef();
	def.meshVertices = wall->vertices;
	def.meshIndices = wall->indices;
	def.meshVertexCount = WALL_VERTEX_COUNT;
	def.meshTriangleCount = WALL_TRIANGLE_COUNT;
	def.piecePoints = pieces->points;
	def.pieceOffsets = pieces->offsets;
	def.pieceCount = PIECE_COUNT;
	def.maxConcavity = 0.05f;
	return def;
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
	ENSURE( first->pieceCount == second->pieceCount );
	ENSURE( first->maxConcavity == second->maxConcavity );
	for ( int i = 0; i <= first->pieceCount; ++i )
	{
		ENSURE( first->offsets[i] == second->offsets[i] );
	}

	for ( int i = 0; i < first->pieceCount; ++i )
	{
		ENSURE( first->concavities[i] == second->concavities[i] );
	}

	for ( int i = 0; i < first->offsets[first->pieceCount]; ++i )
	{
		ENSURE( first->points[i].x == second->points[i].x );
		ENSURE( first->points[i].y == second->points[i].y );
		ENSURE( first->points[i].z == second->points[i].z );
	}

	b3DestroyConvexMergeResult( first );
	b3DestroyConvexMergeResult( second );
	return 0;
}

// A sweep merges at a series of loosening limits in one pass. Each step matches or undercuts the
// hull count of the one before, and stays within its own limit.
static int SweepLoosensStepByStep( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	const float limits[4] = { 0.0f, 0.05f, 0.5f, 1.5f };
	b3ConvexMergeResult* results[4] = { NULL, NULL, NULL, NULL };
	ENSURE( b3SweepConvexPieces( &def, limits, 4, results ) );

	for ( int i = 0; i < 4; ++i )
	{
		ENSURE( results[i] != NULL );
		ENSURE( results[i]->pieceCount >= 1 );
		ENSURE( results[i]->maxConcavity <= limits[i] + 1e-3f );
		if ( i > 0 )
		{
			ENSURE( results[i]->pieceCount <= results[i - 1]->pieceCount );
		}
	}

	// The doorway is 1 wide: it stays open up to half a meter and closes at 1.5
	ENSURE( results[0]->pieceCount == 3 );
	ENSURE( results[1]->pieceCount == 3 );
	ENSURE( IsCovered( results[2], ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) == false );
	ENSURE( results[3]->pieceCount == 1 );
	ENSURE( IsCovered( results[3], ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } ) );

	// The first step is the merge at that limit alone, hull for hull
	def.maxConcavity = limits[0];
	b3ConvexMergeResult* single = b3MergeConvexPieces( &def );
	ENSURE( single != NULL );
	ENSURE( single->pieceCount == results[0]->pieceCount );
	for ( int i = 0; i < single->offsets[single->pieceCount]; ++i )
	{
		ENSURE( single->points[i].x == results[0]->points[i].x );
		ENSURE( single->points[i].y == results[0]->points[i].y );
		ENSURE( single->points[i].z == results[0]->points[i].z );
	}

	b3DestroyConvexMergeResult( single );
	for ( int i = 0; i < 4; ++i )
	{
		b3DestroyConvexMergeResult( results[i] );
	}

	return 0;
}

// A hull budget holds at every step of a sweep.
static int SweepMeetsHullBudget( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	def.maxHullCount = 2;
	const float limits[2] = { 0.01f, 0.05f };
	b3ConvexMergeResult* results[2] = { NULL, NULL };
	ENSURE( b3SweepConvexPieces( &def, limits, 2, results ) );
	ENSURE( results[0]->pieceCount == 2 );
	ENSURE( results[1]->pieceCount == 2 );
	ENSURE( results[0]->maxConcavity > 0.25f );

	b3DestroyConvexMergeResult( results[0] );
	b3DestroyConvexMergeResult( results[1] );
	return 0;
}

static int SweepRejectsBadInput( void )
{
	WallMesh wall;
	WallPieces pieces;
	MakeWall( &wall, false );
	MakePieces( &pieces );

	b3ConvexMergeDef def = MakeWallDef( &wall, &pieces );
	const float limits[2] = { 0.05f, 0.5f };
	b3ConvexMergeResult* results[2] = { NULL, NULL };

	ENSURE( b3SweepConvexPieces( NULL, limits, 2, results ) == false );
	ENSURE( b3SweepConvexPieces( &def, NULL, 2, results ) == false );
	ENSURE( b3SweepConvexPieces( &def, limits, 0, results ) == false );
	ENSURE( b3SweepConvexPieces( &def, limits, 2, NULL ) == false );

	// Limits that tighten, a negative one, and one that is not a number
	const float tightening[2] = { 0.5f, 0.05f };
	ENSURE( b3SweepConvexPieces( &def, tightening, 2, results ) == false );
	const float negative[2] = { -0.1f, 0.05f };
	ENSURE( b3SweepConvexPieces( &def, negative, 2, results ) == false );
	const float notNumber[2] = { 0.05f, NAN };
	ENSURE( b3SweepConvexPieces( &def, notNumber, 2, results ) == false );

	b3ConvexMergeDef bad = def;
	bad.piecePoints = NULL;
	ENSURE( b3SweepConvexPieces( &bad, limits, 2, results ) == false );

	// Nothing was handed out
	ENSURE( results[0] == NULL && results[1] == NULL );
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
	RUN_SUBTEST( SweepLoosensStepByStep );
	RUN_SUBTEST( SweepMeetsHullBudget );
	RUN_SUBTEST( SweepRejectsBadInput );

	return 0;
}

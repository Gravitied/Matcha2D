/**
 * Matcha2D WASM Bridge Implementation
 *
 * Bridges Matcha2D's Structure-of-Arrays (SoA) buffer format to Box2D's
 * object-based API. Maintains a mapping between JS body indices and
 * Box2D body IDs.
 *
 * Memory layout:
 * - World state is stored in a WorldState struct, accessed by handle
 * - Body ID mapping uses a flat int array indexed by JS body index
 * - Shape state is tracked per-body to avoid recreating shapes each frame
 */

#include "wasm_bridge.h"
#include "box2d/box2d.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ── Constants ─────────────────────────────────────────────────────────────── */

#define MAX_WORLDS 4
#define MAX_VERTICES_PER_SHAPE 16

/* Body flag bits — must match packages/types/src/buffers.ts */
#define FLAG_ACTIVE  0x01
#define FLAG_STATIC  0x02
#define FLAG_SLEEPING 0x04
#define FLAG_SENSOR  0x08

/* Shape types — must match packages/types/src/buffers.ts */
#define SHAPE_BOX     0
#define SHAPE_CIRCLE  1
#define SHAPE_POLYGON 2

/* ── World State ───────────────────────────────────────────────────────────── */

typedef struct {
    b2WorldId world_id;
    int capacity;
    /* JS body index -> Box2D body ID mapping */
    b2BodyId *js_to_b2;
    /* Track which JS indices have been created */
    uint8_t *body_created;
    /* Track shape types to detect changes */
    uint8_t *prev_shape_type;
} WorldState;

static WorldState g_worlds[MAX_WORLDS];
static int g_world_count = 0;

/* ── Helper: find world by handle ──────────────────────────────────────────── */

static WorldState *get_world(int handle) {
    if (handle < 0 || handle >= g_world_count) return NULL;
    if (!b2World_IsValid(g_worlds[handle].world_id)) return NULL;
    return &g_worlds[handle];
}

/* ── Helper: convert angle to Box2D rotation ───────────────────────────────── */

static b2Rot make_rot(float angle) {
    b2Rot r;
    r.c = cosf(angle);
    r.s = sinf(angle);
    return r;
}

/* ── b2_init ───────────────────────────────────────────────────────────────── */

WASM_EXPORT int b2_init(float gravityX, float gravityY, int maxBodies) {
    if (g_world_count >= MAX_WORLDS) return -1;

    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity.x = gravityX;
    worldDef.gravity.y = gravityY;

    b2WorldId wid = b2CreateWorld(&worldDef);
    if (!b2World_IsValid(wid)) return -1;

    int handle = g_world_count;
    WorldState *ws = &g_worlds[handle];
    ws->world_id = wid;
    ws->capacity = maxBodies;

    /* Allocate mapping arrays */
    ws->js_to_b2 = (b2BodyId *)calloc((size_t)maxBodies, sizeof(b2BodyId));
    ws->body_created = (uint8_t *)calloc((size_t)maxBodies, sizeof(uint8_t));
    ws->prev_shape_type = (uint8_t *)calloc((size_t)maxBodies, sizeof(uint8_t));

    if (!ws->js_to_b2 || !ws->body_created || !ws->prev_shape_type) {
        free(ws->js_to_b2);
        free(ws->body_created);
        free(ws->prev_shape_type);
        ws->js_to_b2 = NULL;
        return -1;
    }

    g_world_count++;
    return handle;
}

/* ── b2_destroy ────────────────────────────────────────────────────────────── */

WASM_EXPORT void b2_destroy(int worldHandle) {
    WorldState *ws = get_world(worldHandle);
    if (!ws) return;

    b2DestroyWorld(ws->world_id);
    free(ws->js_to_b2);
    free(ws->body_created);
    free(ws->prev_shape_type);

    ws->world_id = b2_nullWorldId;
    ws->js_to_b2 = NULL;
    ws->body_created = NULL;
    ws->prev_shape_type = NULL;
    ws->capacity = 0;
}

/* ── b2_sync_bodies ────────────────────────────────────────────────────────── */

WASM_EXPORT void b2_sync_bodies(
    int worldHandle,
    float *posX, float *posY,
    float *velX, float *velY,
    float *angle, float *angVel,
    float *mass, float *invMass,
    float *inertia, float *invInertia,
    uint8_t *flags,
    int count
) {
    WorldState *ws = get_world(worldHandle);
    if (!ws) return;

    for (int i = 0; i < count; i++) {
        uint8_t flag = flags[i];

        /* Skip inactive and sleeping bodies */
        if (!(flag & FLAG_ACTIVE)) {
            /* If body was previously created, destroy it */
            if (ws->body_created[i]) {
                b2DestroyBody(ws->js_to_b2[i]);
                ws->body_created[i] = 0;
                ws->js_to_b2[i] = b2_nullBodyId;
            }
            continue;
        }

        /* Determine body type from flags */
        b2BodyType bodyType = (flag & FLAG_STATIC) ? b2_staticBody : b2_dynamicBody;

        if (!ws->body_created[i]) {
            /* Create new Box2D body */
            b2BodyDef bodyDef = b2DefaultBodyDef();
            bodyDef.type = bodyType;
            bodyDef.position.x = posX[i];
            bodyDef.position.y = posY[i];
            bodyDef.rotation = make_rot(angle[i]);
            bodyDef.linearVelocity.x = velX[i];
            bodyDef.linearVelocity.y = velY[i];
            bodyDef.angularVelocity = angVel[i];

            b2BodyId bid = b2CreateBody(ws->world_id, &bodyDef);
            if (b2Body_IsValid(bid)) {
                ws->js_to_b2[i] = bid;
                ws->body_created[i] = 1;

                /* Set mass properties for dynamic bodies */
                if (bodyType == b2_dynamicBody && mass[i] > 0.0f) {
                    b2MassData md;
                    md.mass = mass[i];
                    md.rotationalInertia = inertia[i];
                    md.center.x = 0.0f;
                    md.center.y = 0.0f;
                    b2Body_SetMassData(bid, md);
                }
            }
        } else {
            /* Update existing body */
            b2BodyId bid = ws->js_to_b2[i];
            if (!b2Body_IsValid(bid)) {
                ws->body_created[i] = 0;
                continue;
            }

            /* Update velocity (position is updated by solver, read back later) */
            b2Vec2 vel = {velX[i], velY[i]};
            b2Body_SetLinearVelocity(bid, vel);
            b2Body_SetAngularVelocity(bid, angVel[i]);

            /* Update body type if changed */
            if (b2Body_GetType(bid) != bodyType) {
                b2Body_SetType(bid, bodyType);
            }
        }
    }
}

/* ── b2_sync_shapes ────────────────────────────────────────────────────────── */

WASM_EXPORT void b2_sync_shapes(
    int worldHandle,
    float *posX, float *posY, float *angle,
    float *halfExtentX, float *halfExtentY,
    uint8_t *shapeType,
    float *shapeRadius,
    uint8_t *shapeVertexCount,
    float *shapeVerticesX, float *shapeVerticesY,
    int count
) {
    WorldState *ws = get_world(worldHandle);
    if (!ws) return;

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 0.2f;

    for (int i = 0; i < count; i++) {
        if (!ws->body_created[i]) continue;

        b2BodyId bid = ws->js_to_b2[i];
        if (!b2Body_IsValid(bid)) continue;

        uint8_t stype = shapeType[i];

        /* Skip if shape type hasn't changed and body already has shapes */
        if (ws->prev_shape_type[i] == stype && stype != 0) {
            continue;
        }

        /* Destroy existing shapes if type changed */
        if (ws->prev_shape_type[i] != 0) {
            int shapeCount = b2Body_GetShapeCount(bid);
            if (shapeCount > 0) {
                b2ShapeId *shapes = (b2ShapeId *)malloc((size_t)shapeCount * sizeof(b2ShapeId));
                int got = b2Body_GetShapes(bid, shapes, shapeCount);
                for (int s = 0; s < got; s++) {
                    b2DestroyShape(shapes[s], false);
                }
                free(shapes);
            }
        }

        ws->prev_shape_type[i] = stype;

        switch (stype) {
        case SHAPE_CIRCLE: {
            b2Circle circle;
            circle.center.x = 0.0f;
            circle.center.y = 0.0f;
            circle.radius = shapeRadius[i];
            b2CreateCircleShape(bid, &shapeDef, &circle);
            break;
        }
        case SHAPE_BOX: {
            b2Polygon box = b2MakeBox(halfExtentX[i], halfExtentY[i]);
            b2CreatePolygonShape(bid, &shapeDef, &box);
            break;
        }
        case SHAPE_POLYGON: {
            uint8_t vcount = shapeVertexCount[i];
            if (vcount > 0 && vcount <= MAX_VERTICES_PER_SHAPE) {
                b2Polygon poly;
                poly.count = vcount;
                for (int v = 0; v < vcount; v++) {
                    poly.vertices[v].x = shapeVerticesX[i * MAX_VERTICES_PER_SHAPE + v];
                    poly.vertices[v].y = shapeVerticesY[i * MAX_VERTICES_PER_SHAPE + v];
                }
                /* Compute centroid */
                poly.centroid.x = 0.0f;
                poly.centroid.y = 0.0f;
                for (int v = 0; v < vcount; v++) {
                    poly.centroid.x += poly.vertices[v].x;
                    poly.centroid.y += poly.vertices[v].y;
                }
                if (vcount > 0) {
                    poly.centroid.x /= (float)vcount;
                    poly.centroid.y /= (float)vcount;
                }
                b2CreatePolygonShape(bid, &shapeDef, &poly);
            }
            break;
        }
        default:
            break;
        }

        /* Apply mass from shapes after creating them */
        if (b2Body_GetType(bid) == b2_dynamicBody) {
            b2Body_ApplyMassFromShapes(bid);
        }
    }
}

/* ── b2_step ───────────────────────────────────────────────────────────────── */

WASM_EXPORT void b2_step(int worldHandle, float dt, int subSteps) {
    WorldState *ws = get_world(worldHandle);
    if (!ws) return;

    b2World_Step(ws->world_id, dt, subSteps);
}

/* ── b2_read_bodies ────────────────────────────────────────────────────────── */

WASM_EXPORT void b2_read_bodies(
    int worldHandle,
    float *posX, float *posY,
    float *velX, float *velY,
    float *angle, float *angVel,
    int count
) {
    WorldState *ws = get_world(worldHandle);
    if (!ws) return;

    for (int i = 0; i < count; i++) {
        if (!ws->body_created[i]) continue;

        b2BodyId bid = ws->js_to_b2[i];
        if (!b2Body_IsValid(bid)) continue;

        b2Vec2 pos = b2Body_GetPosition(bid);
        b2Rot rot = b2Body_GetRotation(bid);
        b2Vec2 vel = b2Body_GetLinearVelocity(bid);
        float av = b2Body_GetAngularVelocity(bid);

        posX[i] = pos.x;
        posY[i] = pos.y;

        /* Convert rotation (c,s) back to angle */
        angle[i] = atan2f(rot.s, rot.c);

        velX[i] = vel.x;
        velY[i] = vel.y;
        angVel[i] = av;
    }
}

/* ── b2_get_contact_count ──────────────────────────────────────────────────── */

WASM_EXPORT int b2_get_contact_count(int worldHandle) {
    WorldState *ws = get_world(worldHandle);
    if (!ws) return 0;

    b2Counters counters = b2World_GetCounters(ws->world_id);
    return counters.contactCount;
}

/* ── b2_get_contacts ───────────────────────────────────────────────────────── */

WASM_EXPORT void b2_get_contacts(
    int worldHandle,
    int *bodyA, int *bodyB,
    float *nx, float *ny,
    float *px, float *py,
    float *penetration,
    int maxContacts
) {
    WorldState *ws = get_world(worldHandle);
    if (!ws) return;

    /*
     * Build reverse mapping: Box2D body index -> JS body index
     * This is needed because contact data gives us Box2D body IDs
     * but we need to report JS body indices.
     */
    int *b2_to_js = (int *)calloc((size_t)ws->capacity, sizeof(int));
    for (int i = 0; i < ws->capacity; i++) {
        if (ws->body_created[i] && b2Body_IsValid(ws->js_to_b2[i])) {
            b2_to_js[ws->js_to_b2[i].index1] = i;
        }
    }

    int filled = 0;

    /* Iterate through all bodies to find contacts */
    for (int i = 0; i < ws->capacity && filled < maxContacts; i++) {
        if (!ws->body_created[i]) continue;

        b2BodyId bid = ws->js_to_b2[i];
        if (!b2Body_IsValid(bid)) continue;

        int capacity = b2Body_GetContactCapacity(bid);
        if (capacity <= 0) continue;

        b2ContactData *contacts = (b2ContactData *)malloc((size_t)capacity * sizeof(b2ContactData));
        int count = b2Body_GetContactData(bid, contacts, capacity);

        for (int c = 0; c < count && filled < maxContacts; c++) {
            b2ShapeId shapeA = contacts[c].shapeIdA;
            b2ShapeId shapeB = contacts[c].shapeIdB;

            b2BodyId otherA = b2Shape_GetBody(shapeA);
            b2BodyId otherB = b2Shape_GetBody(shapeB);

            /* Find the JS index for the other body */
            int otherIdx = -1;
            if (otherA.index1 != bid.index1) {
                otherIdx = b2_to_js[otherA.index1];
            } else if (otherB.index1 != bid.index1) {
                otherIdx = b2_to_js[otherB.index1];
            }

            if (otherIdx < 0 || otherIdx >= ws->capacity) continue;

            /* Only record each contact once (smaller index first) */
            if (i >= otherIdx) continue;

            bodyA[filled] = i;
            bodyB[filled] = otherIdx;

            /* Get contact manifold data */
            b2Manifold *m = &contacts[c].manifold;
            if (m->pointCount > 0) {
                nx[filled] = m->normal.x;
                ny[filled] = m->normal.y;
                px[filled] = m->points[0].point.x;
                py[filled] = m->points[0].point.y;
                penetration[filled] = m->points[0].separation;
            } else {
                nx[filled] = 0.0f;
                ny[filled] = 0.0f;
                px[filled] = 0.0f;
                py[filled] = 0.0f;
                penetration[filled] = 0.0f;
            }

            filled++;
        }

        free(contacts);
    }

    free(b2_to_js);
}

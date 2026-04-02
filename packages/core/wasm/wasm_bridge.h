/**
 * Matcha2D WASM Bridge
 *
 * C API bridging Matcha2D's SoA buffer format to Box2D internals.
 * Compiled to WASM via Emscripten, called from TypeScript wrappers.
 *
 * All exported functions use EMSCRIPTEN_KEEPALIVE to ensure they are
 * included in the WASM module's export table.
 */

#ifndef MATCHA2D_WASM_BRIDGE_H
#define MATCHA2D_WASM_BRIDGE_H

#include <stdint.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define WASM_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define WASM_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create a Box2D simulation world.
 *
 * @param gravityX  Gravity X component (usually 0)
 * @param gravityY  Gravity Y component (usually -9.81)
 * @param maxBodies Maximum body capacity for internal mapping arrays
 * @return          World handle (positive int) or -1 on failure
 */
WASM_EXPORT int b2_init(float gravityX, float gravityY, int maxBodies);

/**
 * Destroy a simulation world and free all associated memory.
 *
 * @param worldHandle Handle returned by b2_init
 */
WASM_EXPORT void b2_destroy(int worldHandle);

/**
 * Sync body state from JavaScript SoA buffers into Box2D.
 *
 * For each active body: creates a new Box2D body if it doesn't exist yet,
 * or updates position/velocity for existing bodies. Uses the flags array
 * to determine which bodies are ACTIVE (0x01) or STATIC (0x02).
 *
 * @param worldHandle  Handle returned by b2_init
 * @param posX         SoA: body positions X
 * @param posY         SoA: body positions Y
 * @param velX         SoA: body velocities X
 * @param velY         SoA: body velocities Y
 * @param angle        SoA: body rotation angles (radians)
 * @param angVel       SoA: body angular velocities
 * @param mass         SoA: body masses
 * @param invMass      SoA: precomputed inverse masses (0 = static)
 * @param inertia      SoA: rotational inertias
 * @param invInertia   SoA: precomputed inverse inertias
 * @param flags        SoA: body flags (ACTIVE=0x01, STATIC=0x02)
 * @param count        Number of bodies in the buffers
 */
WASM_EXPORT void b2_sync_bodies(
    int worldHandle,
    float *posX, float *posY,
    float *velX, float *velY,
    float *angle, float *angVel,
    float *mass, float *invMass,
    float *inertia, float *invInertia,
    uint8_t *flags,
    int count
);

/**
 * Sync shape state from JavaScript SoA buffers into Box2D.
 *
 * Creates or updates Box2D shapes (circles, boxes, polygons) on the
 * corresponding bodies. Shape data is read from the SoA buffers and
 * matched to bodies by index.
 *
 * Shape types: 0 = Box, 1 = Circle, 2 = Polygon
 *
 * @param worldHandle      Handle returned by b2_init
 * @param posX             SoA: body positions X (for shape center)
 * @param posY             SoA: body positions Y
 * @param angle            SoA: body rotation angles
 * @param halfExtentX      SoA: AABB half-extents X (for box shapes)
 * @param halfExtentY      SoA: AABB half-extents Y (for box shapes)
 * @param shapeType        SoA: shape type per body (0=Box, 1=Circle, 2=Polygon)
 * @param shapeRadius      SoA: circle radius (when shapeType == Circle)
 * @param shapeVertexCount SoA: vertex count (when shapeType == Polygon)
 * @param shapeVerticesX   SoA: polygon vertex X coordinates (capacity * MAX_VERTICES)
 * @param shapeVerticesY   SoA: polygon vertex Y coordinates (capacity * MAX_VERTICES)
 * @param count            Number of bodies
 */
WASM_EXPORT void b2_sync_shapes(
    int worldHandle,
    float *posX, float *posY, float *angle,
    float *halfExtentX, float *halfExtentY,
    uint8_t *shapeType,
    float *shapeRadius,
    uint8_t *shapeVertexCount,
    float *shapeVerticesX, float *shapeVerticesY,
    int count
);

/**
 * Advance the simulation by one time step.
 *
 * @param worldHandle Handle returned by b2_init
 * @param dt          Time step duration (usually 1/60)
 * @param subSteps    Number of sub-steps for accuracy (usually 4)
 */
WASM_EXPORT void b2_step(int worldHandle, float dt, int subSteps);

/**
 * Read simulation results back into JavaScript SoA buffers.
 *
 * After b2_step, this reads the updated positions, velocities, angles,
 * and angular velocities from Box2D bodies back into the provided arrays.
 *
 * @param worldHandle Handle returned by b2_init
 * @param posX        SoA output: updated positions X
 * @param posY        SoA output: updated positions Y
 * @param velX        SoA output: updated velocities X
 * @param velY        SoA output: updated velocities Y
 * @param angle       SoA output: updated angles
 * @param angVel      SoA output: updated angular velocities
 * @param count       Number of bodies to read
 */
WASM_EXPORT void b2_read_bodies(
    int worldHandle,
    float *posX, float *posY,
    float *velX, float *velY,
    float *angle, float *angVel,
    int count
);

/**
 * Get the number of active contact pairs.
 *
 * @param worldHandle Handle returned by b2_init
 * @return            Number of contacts
 */
WASM_EXPORT int b2_get_contact_count(int worldHandle);

/**
 * Fill arrays with contact data for all active contacts.
 *
 * @param worldHandle  Handle returned by b2_init
 * @param bodyA        Output: JS body index for contact side A
 * @param bodyB        Output: JS body index for contact side B
 * @param nx           Output: contact normal X components
 * @param ny           Output: contact normal Y components
 * @param px           Output: contact point X coordinates
 * @param py           Output: contact point Y coordinates
 * @param penetration  Output: penetration depths
 * @param maxContacts  Maximum number of contacts to fill
 */
WASM_EXPORT void b2_get_contacts(
    int worldHandle,
    int *bodyA, int *bodyB,
    float *nx, float *ny,
    float *px, float *py,
    float *penetration,
    int maxContacts
);

#ifdef __cplusplus
}
#endif

#endif /* MATCHA2D_WASM_BRIDGE_H */

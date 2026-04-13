import { MatchaBuffers, NarrowphaseColliderOutline } from '@matcha2d/types';

/**
 * Abstract renderer interface.
 * Dev B implements Canvas2D and WebGL backends.
 */
interface IRenderer {
    /** Begin a new frame. */
    begin(): void;
    /** Draw all bodies from the buffer data. */
    drawBodies(buffers: MatchaBuffers, count: number, alpha: number): void;
    /** End the frame and flush to screen. */
    end(): void;
    /** Clean up resources. */
    destroy(): void;
}

/**
 * Canvas2D debug renderer.
 * Draws oriented bodies (circles, boxes, polygons) with proper rotation.
 * Following the article's guidance: shapes are drawn in world space by
 * applying rotation matrices to transform from model space to world space.
 */
declare class Canvas2DRenderer implements IRenderer {
    private readonly _ctx;
    private _scale;
    private _offsetX;
    private _offsetY;
    constructor(_ctx: CanvasRenderingContext2D, options?: {
        scale?: number;
        offsetX?: number;
        offsetY?: number;
    });
    /** Set the camera scale (pixels per meter). */
    setScale(scale: number): void;
    /** Set the camera offset in pixels. */
    setOffset(x: number, y: number): void;
    begin(): void;
    drawBodies(buffers: MatchaBuffers, count: number, _alpha: number): void;
    end(): void;
    /**
     * Draw narrowphase-accurate collider outlines in **world meters** (Y-up).
     * Call between `begin()` and `end()` so the same transform as `drawBodies` applies.
     */
    drawNarrowphaseColliderOutlinesMeters(outlines: NarrowphaseColliderOutline[], options?: {
        lineWidth?: number;
        strokeSolid?: string;
        strokeSensor?: string;
    }): void;
    destroy(): void;
    private _drawBox;
    private _drawCircle;
    private _drawPolygon;
}

interface DrawNarrowphaseCanvasOptions {
    /** Pixels per physics meter (same as demo `PIXELS_PER_METER`; use `1` if physics units are already pixels) */
    pixelsPerMeter: number;
    /** Canvas height in pixels — used when `physicsYUp` is true */
    canvasHeight: number;
    /**
     * If true (default), world space is Y-up (Matcha demos in meters) → map with `H - y * ppm`.
     * If false, world Y matches canvas Y-down (e.g. `collision.html` pixel space) → `y * ppm` only.
     */
    physicsYUp?: boolean;
    strokeSolid?: string;
    strokeSensor?: string;
    /** Stroke width in **canvas pixels** */
    lineWidth?: number;
}
/**
 * Draw narrowphase outlines on a **raw** canvas context (Y-down, pixels).
 * With default `physicsYUp: true`, matches `demo/compare.html` (`toCanvasX` / `toCanvasY`).
 */
declare function drawNarrowphaseColliderOutlinesCanvas(ctx: CanvasRenderingContext2D, outlines: NarrowphaseColliderOutline[], opts: DrawNarrowphaseCanvasOptions): void;

export { Canvas2DRenderer, type DrawNarrowphaseCanvasOptions, type IRenderer, drawNarrowphaseColliderOutlinesCanvas };

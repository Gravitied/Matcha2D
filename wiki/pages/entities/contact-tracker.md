---
title: "Contact Tracker"
type: entity
tags: [collision, api, dev-a]
related: [collision-pipeline.md, world-class.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Contact Tracker

## Location

`packages/core/src/collision/contact-tracker.ts`

## What It Is

Tracks contact state across frames to fire begin/stay/end events. Compares the manifold set from the current frame to the previous frame.

## Events

| Event | Fires when |
|-------|-----------|
| `onBegin` | A contact pair appears that wasn't present last frame |
| `onStay` | A contact pair is present in both current and previous frame |
| `onEnd` | A contact pair was present last frame but is gone now |

## Usage

`World.setCollisionCallbacks({ onBegin?, onStay?, onEnd? })` — registers callbacks consumed by the tracker.

Each callback receives the two `BodyHandle`s involved.

## Related

- [[collision-pipeline]] — produces the manifolds ContactTracker compares
- [[world-class]] — exposes `setCollisionCallbacks`

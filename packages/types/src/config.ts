export interface WorldConfig {
  gravity: { x: number; y: number }
  fixedTimestep: number
  velocityIterations: number
  positionIterations: number
  maxBodies: number
  sleepEnabled: boolean
  sleepTimeThreshold: number
  sleepVelocityThreshold: number
}

export const DEFAULT_WORLD_CONFIG: Readonly<WorldConfig> = {
  gravity: { x: 0, y: -9.81 },
  fixedTimestep: 1 / 60,
  velocityIterations: 8,
  positionIterations: 3,
  maxBodies: 8192,
  sleepEnabled: true,
  sleepTimeThreshold: 0.5,
  sleepVelocityThreshold: 0.01,
}

export class Pose {
  constructor(public x: number, public y: number, public yaw: number) {}

  get yawDeg(): number {
    return (this.yaw / Math.PI) * 180.0;
  }

  static from_dict(p: any): Pose {
    return new Pose(p.x, p.y, p.yaw.rad);
  }
}

export class Pose {
  constructor(public x: number, public y: number, public yaw: number) {}

  static fromDict(p: any): Pose {
    return new Pose(p.x, p.y, p.yaw);
  }

  get yawDeg(): number {
    return (this.yaw / Math.PI) * 180.0;
  }
}

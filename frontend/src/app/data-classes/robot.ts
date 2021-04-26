import { Pose } from "./pose";
import { Velocity } from "./velocity";

export class Robot {
  constructor(
    public velocity: Velocity,
    public pose: Pose,
    public battery: number,
    public temperature: number
  ) {}

  static fromDict(r: any): Robot {
    return new Robot(
      Velocity.fromDict(r.velocity),
      Pose.fromDict(r.pose),
      r.battery,
      r.temperature
    );
  }
}

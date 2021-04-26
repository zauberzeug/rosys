import { Mode } from "./mode";
import { Robot } from "./robot";

export class World {
  constructor(public mode: Mode, public time: number, public robot: Robot) {}

  static fromDict(w: any): World {
    return new World(w.mode, w.time, Robot.fromDict(w.robot));
  }
}

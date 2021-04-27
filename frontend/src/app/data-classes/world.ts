import { Mode } from "./mode";
import { Robot } from "./robot";
import { State } from "./state";

export class World {
  constructor(
    public mode: Mode,
    public state: State,
    public time: number,
    public robot: Robot
  ) {}

  static fromDict(w: any): World {
    return new World(w.mode, w.state, w.time, Robot.fromDict(w.robot));
  }
}

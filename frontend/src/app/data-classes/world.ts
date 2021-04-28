import { Camera } from "./camera";
import { Mode } from "./mode";
import { Robot } from "./robot";
import { State } from "./state";

export class World {
  constructor(
    public mode: Mode,
    public state: State,
    public time: number,
    public robot: Robot,
    public cameras: { [mac: string]: Camera }
  ) {}

  static fromDict(w: any): World {
    const cameras: { [mac: string]: Camera } = {};
    Object.values(w.cameras).forEach(
      (c: any) => (cameras[c["mac"]] = Camera.fromDict(c))
    );
    return new World(w.mode, w.state, w.time, Robot.fromDict(w.robot), cameras);
  }
}

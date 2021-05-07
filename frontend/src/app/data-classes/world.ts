import { Camera } from "./camera";
import { Image } from "./image";
import { Mode } from "./mode";
import { Robot } from "./robot";
import { State } from "./state";

export class World {
  constructor(
    public mode: Mode,
    public state: State,
    public time: number,
    public robot: Robot,
    public cameras: { [mac: string]: Camera },
    public images: Image[]
  ) {}

  static fromDict(w: any): World {
    const cameras: { [mac: string]: Camera } = {};
    Object.values(w.cameras).forEach(
      (c: any) => (cameras[c["mac"]] = Camera.fromDict(c))
    );
    return new World(
      w.mode,
      w.state,
      w.time,
      Robot.fromDict(w.robot),
      cameras,
      w.images.map((i: any) => Image.fromDict(i))
    );
  }

  updateFromDict(w: any) {
    if (w.mode) this.mode = w.mode;
    if (w.state) this.state = w.state;
    if (w.time) this.time = w.time;
    if (w.robot) this.robot = Robot.fromDict(w.robot);
    if (w.cameras) {
      const cameras: { [mac: string]: Camera } = {};
      Object.values(w.cameras).forEach(
        (c: any) => (cameras[c["mac"]] = Camera.fromDict(c))
      );
    }
    if (w.images) this.images = w.images.map((i: any) => Image.fromDict(i));
  }
}

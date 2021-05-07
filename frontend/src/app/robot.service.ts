import { Injectable } from "@angular/core";
import { Camera } from "./data-classes/camera";
import { World } from "./data-classes/world";
import { WrappedSocket } from "./socket-io/socket-io.service";

@Injectable({
  providedIn: "root",
})
export class RobotService {
  world?: World;

  get cameras(): Array<Camera> {
    return Object.values(this.world?.cameras || {});
  }

  constructor(private socket: WrappedSocket) {
    socket.on("world", (data: any) => (this.world = World.fromDict(data)));
    socket.on("world_part", (data: any) => this.world?.updateFromDict(data));
  }

  sendPower(left: number, right: number) {
    this.socket.emit("drive_power", { left: left, right: right });
  }

  sendTask(task: string) {
    this.socket.emit("task", task);
  }

  sendPause() {
    this.socket.emit("pause");
  }

  sendResume() {
    this.socket.emit("resume");
  }
}

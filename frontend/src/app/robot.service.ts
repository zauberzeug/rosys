import { Injectable } from "@angular/core";
import { World } from "./data-classes/world";
import { WrappedSocket } from "./socket-io/socket-io.service";

@Injectable({
  providedIn: "root",
})
export class RobotService {
  world?: World;

  constructor(private socket: WrappedSocket) {
    socket.on("world", (data: any) => (this.world = World.fromDict(data)));
  }

  sendPower(left: number, right: number) {
    this.socket.emit("drive_power", { left: left, right: right });
  }

  sendTask(task: string) {
    this.socket.emit("task", task);
  }
}

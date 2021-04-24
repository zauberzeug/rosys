import { Injectable } from "@angular/core";
import { Pose } from "./home/pose";
import { WrappedSocket } from "./socket-io/socket-io.service";

@Injectable({
  providedIn: "root",
})
export class RobotService {
  mode?: number;
  time: number = 0;
  pose?: Pose;
  width: number = 1.0;

  constructor(private socket: WrappedSocket) {
    socket.on("world", (data: any) => {
      this.mode = data.mode;
      this.width = data.robot.width;
      this.pose = Pose.from_dict(data.robot.pose);
      this.time = data.time;
    });
  }

  sendPower(left: number, right: number) {
    this.socket.emit("drive_power", { left: left, right: right });
  }

  sendTask(task: string) {
    this.socket.emit("task", task);
  }
}

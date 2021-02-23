import {
  Component,
  Input,
  OnInit,
  OnChanges,
  SimpleChanges,
} from "@angular/core";
import * as uuid from "uuid";
import * as THREE from "three";
import { MapComponent } from "./map.component";

@Component({
  selector: "app-object3d",
  template: "",
})
export class Object3dComponent implements OnInit, OnChanges {
  id!: string;
  @Input() x: number = 0;
  @Input() y: number = 0;
  @Input() z: number = 0;

  object3d!: THREE.Object3D;
  container!: MapComponent;

  ngOnInit() {
    this.id = uuid.v4();
    this.object3d.uuid = this.id;
    this.object3d.position.set(this.x, this.y, this.z);
  }

  ngOnChanges(changes: SimpleChanges) {
    if (!this.object3d) return;
    for (const key in changes) {
      const value = Number(changes[key].currentValue);
      if (key == "x") this.object3d.position.setX(value);
      if (key == "y") this.object3d.position.setY(value);
      if (key == "z") this.object3d.position.setZ(value);
    }
  }

  invalidate() {
    if (this.container) this.container.resync();
  }
}

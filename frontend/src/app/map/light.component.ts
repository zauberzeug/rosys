import { Component, OnInit } from "@angular/core";
import { Object3dComponent } from "./object3d.component";
import * as THREE from "three";

@Component({
  selector: "app-light",
  template: "",
  providers: [{ provide: Object3dComponent, useExisting: LightComponent }],
})
export class LightComponent extends Object3dComponent implements OnInit {
  ngOnInit() {
    const group = new THREE.Group();

    group.add(new THREE.AmbientLight(0xffffff, 0.7));

    var light = new THREE.DirectionalLight(0xffffff, 0.3);
    light.position.set(this.x, this.y, this.z);
    light.castShadow = true;
    light.shadow.mapSize.width = 4096;
    light.shadow.mapSize.height = 4096;
    var d = 50;
    light.shadow.camera.left = -d;
    light.shadow.camera.right = d;
    light.shadow.camera.top = d;
    light.shadow.camera.bottom = -d;
    light.shadow.camera.far = 100;
    group.add(light);

    this.object3d = group;

    super.ngOnInit();
  }
}

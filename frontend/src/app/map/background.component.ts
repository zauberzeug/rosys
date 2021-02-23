import { Component, OnInit } from "@angular/core";
import * as THREE from "three";
import { Object3dComponent } from "./object3d.component";

@Component({
  selector: "app-background",
  template: "",
  providers: [{ provide: Object3dComponent, useExisting: BackgroundComponent }],
})
export class BackgroundComponent extends Object3dComponent implements OnInit {
  ngOnInit() {
    const group = new THREE.Group();

    group.add(new THREE.AxesHelper(1));

    const groundGeo = new THREE.PlaneGeometry(10000, 10000);
    const groundMat = new THREE.MeshPhongMaterial();
    groundMat.color.setHSL(0.1, 1, 0.85);
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.receiveShadow = true;
    ground.name = "ground";
    ground.translateZ(-0.1);
    group.add(ground);

    const grid = new THREE.GridHelper(100, 100);
    (grid.material as THREE.LineBasicMaterial).transparent = true;
    (grid.material as THREE.LineBasicMaterial).opacity = 0.2;
    grid.rotateX(Math.PI / 2);
    group.add(grid);

    this.object3d = group;
    this.object3d.position.set(this.x, this.y, this.z);
    super.ngOnInit();
  }
}

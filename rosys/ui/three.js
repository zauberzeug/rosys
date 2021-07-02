var scene;
var camera;
var robot;
var orbitControls;
var images = new Map();
var loader = new THREE.TextureLoader();

Vue.component("three", {
  template: `<canvas v-bind:id="jp_props.id"></div>`,

  mounted() {
    scene = new THREE.Scene();

    const width = 400;
    const height = 300;

    camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.up = new THREE.Vector3(0, 0, 1);
    camera.position.set(0, -3, 5);

    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const light = new THREE.DirectionalLight(0xffffff, 0.3);
    light.position.set(5, 10, 40);
    scene.add(light);

    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      canvas: document.getElementById(this.$props.jp_props.id),
    });
    renderer.setClearColor("#eee");
    renderer.setSize(width, height);

    const grid = new THREE.GridHelper(100, 100);
    grid.material.transparent = true;
    grid.material.opacity = 0.2;
    grid.rotateX(Math.PI / 2);
    scene.add(grid);

    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const material = new THREE.MeshPhongMaterial({ color: "#433F81" });
    robot = new THREE.Mesh(geometry, material);
    scene.add(robot);

    orbitControls = new THREE.OrbitControls(camera, renderer.domElement);

    const render = function () {
      requestAnimationFrame(() => setTimeout(() => render(), 1000 / 20));
      renderer.render(scene, camera);
    };
    render();

    const raycaster = new THREE.Raycaster();
    document.getElementById(this.$props.jp_props.id).onclick = (mouseEvent) => {
      let x = (mouseEvent.offsetX / renderer.domElement.width) * 2 - 1;
      let y = -(mouseEvent.offsetY / renderer.domElement.height) * 2 + 1;
      raycaster.setFromCamera({ x: x, y: y }, camera);
      const event = {
        event_type: "onClick",
        vue_type: this.$props.jp_props.vue_type,
        id: this.$props.jp_props.id,
        page_id: page_id,
        websocket_id: websocket_id,
        objects: raycaster.intersectObjects(scene.children, true),
      };
      send_to_server(event, "event");
    };
  },

  updated() {
    robot.position.x = this.$props.jp_props.options.robot_pose.x;
    robot.position.y = this.$props.jp_props.options.robot_pose.y;
    robot.setRotationFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      this.$props.jp_props.options.robot_pose.yaw
    );

    if (this.$props.jp_props.options.follow_robot) {
      const target = new THREE.Vector3(robot.position.x, robot.position.y, 0);
      orbitControls.target = orbitControls.target
        .clone()
        .multiplyScalar(0.9)
        .add(target.multiplyScalar(0.1));
      camera.lookAt(orbitControls.target);
      camera.updateProjectionMatrix();
    }

    const jp_images = this.$props.jp_props.options.images;
    jp_images.forEach((image) => {
      if (!images.has(image.id)) {
        console.log("add", image.id);
        const projection = image.camera.projection;
        const geometry = new THREE.BufferGeometry();
        const nI = projection[0].length;
        const nJ = projection.length;
        const vertices = [];
        const indices = [];
        const uvs = [];
        for (let j = 0; j < nJ; ++j) {
          for (let i = 0; i < nI; ++i) {
            const X = (projection[j][i] || [0, 0])[0];
            const Y = (projection[j][i] || [0, 0])[1];
            vertices.push(X, Y, 0);
            uvs.push(i / (nI - 1), j / (nJ - 1));
          }
        }
        for (let j = 0; j < nJ - 1; ++j) {
          for (let i = 0; i < nI - 1; ++i) {
            if (
              projection[j][i] &&
              projection[j][i + 1] &&
              projection[j + 1][i] &&
              projection[j + 1][i + 1]
            ) {
              const idx00 = i + j * nI;
              const idx10 = i + j * nI + 1;
              const idx01 = i + j * nI + nI;
              const idx11 = i + j * nI + 1 + nI;
              indices.push(idx00, idx10, idx01);
              indices.push(idx10, idx11, idx01);
            }
          }
        }
        geometry.setIndex(new THREE.Uint32BufferAttribute(indices, 1));
        geometry.setAttribute(
          "position",
          new THREE.Float32BufferAttribute(vertices, 3)
        );
        geometry.setAttribute("uv", new THREE.Float32BufferAttribute(uvs, 2));
        geometry.computeVertexNormals();
        geometry.computeFaceNormals();
        const texture = loader.load(image.data);
        texture.flipY = false;
        texture.minFilter = THREE.LinearFilter;
        const material = new THREE.MeshLambertMaterial({
          map: texture,
          side: THREE.DoubleSide,
        });
        const mesh = new THREE.Mesh(geometry, material);
        scene.add(mesh);
        images.set(image.id, mesh);
      }
    });
    images.forEach((_, id) => {
      if (!jp_images.some((image) => image.id == id)) {
        console.log("remove", id);
        scene.remove(images.get(id));
        images.delete(id);
      }
    });
  },

  props: {
    jp_props: Object,
  },
});

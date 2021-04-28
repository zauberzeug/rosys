class CameraNetwork {
  constructor(
    public ip: string,
    public inactiveTime: number,
    public expectedThroughput: number,
    public signal: number,
    public signalAverage: number
  ) {}

  static fromDict(n: any): CameraNetwork {
    return new CameraNetwork(
      n.ip,
      n.inactive_time,
      n.expected_throughput,
      n.signal,
      n.signal_average
    );
  }
}

export class Camera {
  constructor(public mac: string, public network: CameraNetwork) {}

  static fromDict(c: any): Camera {
    return new Camera(c.mac, CameraNetwork.fromDict(c.network));
  }
}

export class Velocity {
  constructor(public linear: number, public angular: number) {}

  static fromDict(v: any): Velocity {
    return new Velocity(v.linear, v.angular);
  }
}

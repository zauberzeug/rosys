export class Image {
  constructor(public id: string, public time: number, public mac: string) {}

  static fromDict(i: any): Image {
    return new Image(i.id, i.time, i.mac);
  }
}

export class Color {

  static hex = (rgb: number) => new Color((rgb & 0xff0000) >> 16,
                                          (rgb & 0x00ff00) >> 8,
                                          (rgb & 0x0000ff),
                                          255)

  static lerp = (a: Color, b: Color, t: number) => {
    if (t < 0) { t = 0 }
    if (t > 1) { t = 1 }

    return new Color(a.r + (b.r - a.r) * t,
                     a.g + (b.g - a.g) * t,
                     a.b + (b.b - a.b) * t,
                     a.a + (b.a - a.a) * t)
  }

  static white = new Color(255, 255, 255, 255)
  static black = new Color(0, 0, 0, 255)
  static red = new Color(255, 0, 0, 255)


  get rgb() {
    return (this.r << 16) | (this.g << 8) | this.b
  }

  get rgba() {
    return [this.r/255, this.g/255, this.b/255, this.a/255]
  }



  constructor(
    readonly r: number, 
    readonly g: number, 
    readonly b: number, 
    readonly a: number) {}
}
import { Rectangle, Vec2 } from "../math/vec2";

export class SpriteBatch {
  private gl: WebGL2RenderingContext;
  private maxSprites = 1000;
  private vertexSize = 4 * (2 + 2 + 4); // pos (2) + uv (2) + color (4)
  private buffer: Float32Array;
  private bufferIndex = 0;
  private vao: WebGLVertexArrayObject;
  private vbo: WebGLBuffer;
  private shader: WebGLProgram;
  private texture: WebGLTexture | null = null;

  private width: number
  private height: number

  constructor(gl: WebGL2RenderingContext, shader: WebGLProgram, width: number, height: number) {
    this.width = width
    this.height = height
    this.gl = gl;
    this.shader = shader;
    this.buffer = new Float32Array(this.maxSprites * 6 * this.vertexSize / 4);

    this.vao = gl.createVertexArray()!;
    this.vbo = gl.createBuffer()!;

    gl.bindVertexArray(this.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vbo);
    gl.bufferData(gl.ARRAY_BUFFER, this.buffer.byteLength, gl.DYNAMIC_DRAW);

    const stride = this.vertexSize;

    // Position
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, stride, 0);

    // UV
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 2, gl.FLOAT, false, stride, 2 * 4);

    // Color
    gl.enableVertexAttribArray(2);
    gl.vertexAttribPointer(2, 4, gl.FLOAT, false, stride, 4 * 4);
  }

  begin(texture: WebGLTexture) {
    this.bufferIndex = 0;
    this.texture = texture;
  }


  draw_tri(x: number[], y: number[], u: number[], v: number[], color = [1, 1, 1, 1]) {
    if (this.bufferIndex + 6 * this.vertexSize / 4 >= this.buffer.length) {
      this.flush();
    }


    const vertices = [
      [x[0], y[0], u[0], v[0]],
      [x[1], y[1], u[1], v[1]],
      [x[2], y[2], u[2], v[2]],
    ];

    for (const [px, py, pu, pv] of vertices) {
      this.buffer[this.bufferIndex++] = px;
      this.buffer[this.bufferIndex++] = py;
      this.buffer[this.bufferIndex++] = pu;
      this.buffer[this.bufferIndex++] = pv;
      this.buffer[this.bufferIndex++] = color[0];
      this.buffer[this.bufferIndex++] = color[1];
      this.buffer[this.bufferIndex++] = color[2];
      this.buffer[this.bufferIndex++] = color[3];
    }
  }

  draw(x: number, y: number, w: number, h: number, u = 0, v = 0, u2 = 1, v2 = 1, color = [1, 1, 1, 1], theta: number = 0) {
    if (this.bufferIndex + 6 * this.vertexSize / 4 >= this.buffer.length) {
      this.flush();
    }

    let cx = x + w / 2
    let cy = y + h / 2

    const vertices = [
      [...Vec2.rotate_point(x, y, theta, cx, cy).xy, u, v],
      [...Vec2.rotate_point(x + w, y, theta, cx, cy).xy, u2, v],
      [...Vec2.rotate_point(x + w, y + h, theta, cx, cy).xy, u2, v2],

      [...Vec2.rotate_point(x, y, theta, cx, cy).xy, u, v],
      [...Vec2.rotate_point(x + w, y + h, theta, cx, cy).xy, u2, v2],
      [...Vec2.rotate_point(x, y + h, theta, cx, cy).xy, u, v2]
    ];

    for (const [px, py, pu, pv] of vertices) {
      this.buffer[this.bufferIndex++] = px;
      this.buffer[this.bufferIndex++] = py;
      this.buffer[this.bufferIndex++] = pu;
      this.buffer[this.bufferIndex++] = pv;
      this.buffer[this.bufferIndex++] = color[0];
      this.buffer[this.bufferIndex++] = color[1];
      this.buffer[this.bufferIndex++] = color[2];
      this.buffer[this.bufferIndex++] = color[3];
    }
  }

  draw_rect(rect: Rectangle, u = 0, v = 0, u2 = 1, v2 = 1, color = [1, 1, 1, 1]) {
    if (this.bufferIndex + 6 * this.vertexSize / 4 >= this.buffer.length) {
      this.flush();
    }


    let [a, b, c, d] = rect.vertices


    const vertices = [
      [a.x, a.y, u, v],
      [b.x, b.y, u2, v],
      [c.x, c.y, u2, v2],

      [a.x, a.y, u, v],
      [c.x, c.y, u2, v2],
      [d.x, d.y, u, v2]
    ];

    for (const [px, py, pu, pv] of vertices) {
      this.buffer[this.bufferIndex++] = px / this.width
      this.buffer[this.bufferIndex++] = py / this.height
      this.buffer[this.bufferIndex++] = pu;
      this.buffer[this.bufferIndex++] = pv;
      this.buffer[this.bufferIndex++] = color[0];
      this.buffer[this.bufferIndex++] = color[1];
      this.buffer[this.bufferIndex++] = color[2];
      this.buffer[this.bufferIndex++] = color[3];
    }
  }






  flush() {
    const gl = this.gl;
    if (this.bufferIndex === 0 || !this.texture) return;

    gl.useProgram(this.shader);
    gl.bindVertexArray(this.vao);
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vbo);
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, this.buffer.subarray(0, this.bufferIndex));
    gl.drawArrays(gl.TRIANGLES, 0, this.bufferIndex / (this.vertexSize / 4));
    this.bufferIndex = 0;
  }
}

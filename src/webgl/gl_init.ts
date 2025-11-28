import vertexShader from './batch.vert'
import fragmentShader from './batch.frag'

import zoomVertexShader from './zoom.vert'
import zoomFragmentShader from './zoom.frag'

import { createShaderProgram } from './shader';
import { SpriteBatch } from './batch';
import { loadTexture } from './texture';

export type GL = {
    canvas: HTMLCanvasElement,
    load_sheet(image: HTMLImageElement): void
    load_bg(image: HTMLImageElement): void
    load_tiles(image: HTMLImageElement): void
    begin_render(): void
    begin_render_bg(): void
    begin_render_tiles(): void
    draw(x: number, y: number, w: number, h: number, sx: number, sy: number, flip_x: boolean, theta?: number): void
    draw_tri(x: [number, number, number], y: [number, number, number], u: [number, number, number], sy: [number, number, number]): void
    end_render(): void
    flush_to_screen(): void
    clear(): void
    begin_stencil(): void,
    begin_stencil_bg(): void,
    end_stencil(): void
}

export const g = GL(320, 180)


export function GL(width: number, height: number): GL {

    const canvas = document.createElement('canvas');
    document.body.appendChild(canvas);
    canvas.width = width;
    canvas.height = height;

    const gl = canvas.getContext('webgl2', { antialias: false, depth: false, stencil: true })!;


    const shader = createShaderProgram(gl, vertexShader, fragmentShader);
    const batch = new SpriteBatch(gl, shader);
    const screenShader = createShaderProgram(gl, zoomVertexShader, zoomFragmentShader);

    gl.clearColor(130/255, 112/255, 148/255, 1)
    gl.viewport(0, 0, width, height)

    gl.enable(gl.BLEND)
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)

    gl.enable(gl.STENCIL_TEST)

    let texture: WebGLTexture
    let bg_texture: WebGLTexture
    let t_width: number, t_height: number
    let bg_t_width: number, bg_t_height: number
    let t_t_width: number, t_t_height: number

    let tiles_texture: WebGLTexture
    let tiles_t_width: number, tiles_t_height: number


    let batch_render_target = createRenderTarget(gl, width, height)

    let fullscreenQuadVAO = initFullscreenQuad(gl)

    return {
        canvas,
        load_sheet(image: HTMLImageElement) {
            texture = loadTexture(gl, image)
            t_t_width = image.width
            t_t_height = image.height
        },
        load_bg(image: HTMLImageElement) {
            bg_texture = loadTexture(gl, image)
            bg_t_width = image.width
            bg_t_height = image.height
        },
        load_tiles(image: HTMLImageElement) {
            tiles_texture = loadTexture(gl, image)
            tiles_t_width = image.width
            tiles_t_height = image.height
        },
        clear() {
            gl.clear(gl.COLOR_BUFFER_BIT | gl.STENCIL_BUFFER_BIT)
        },
        begin_stencil() {
            gl.clear(gl.STENCIL_BUFFER_BIT)
            gl.colorMask(false, false, false, false)
            gl.stencilFunc(gl.ALWAYS, 1, 0xFF)
            gl.stencilOp(gl.KEEP, gl.KEEP, gl.REPLACE)
        },
        begin_stencil_bg() {
            gl.colorMask(true, true, true, true)
            gl.stencilFunc(gl.EQUAL, 1, 0xFF)
            gl.stencilOp(gl.KEEP, gl.KEEP, gl.KEEP)
        },
        end_stencil() {
            gl.colorMask(true, true, true, true)
            gl.stencilFunc(gl.ALWAYS, 0, 0xFF)
            gl.stencilOp(gl.KEEP, gl.KEEP, gl.KEEP)
        },
        begin_render_bg() {
            //gl.bindFramebuffer(gl.FRAMEBUFFER, batch_render_target.framebuffer)
            //gl.viewport(0, 0, batch_render_target.width, batch_render_target.height)

            t_width = bg_t_width
            t_height = bg_t_height
            batch.begin(bg_texture)
        },
        begin_render_tiles() {

            t_width = tiles_t_width
            t_height = tiles_t_height
            batch.begin(tiles_texture)
        },
        begin_render() {
            t_width = t_t_width
            t_height = t_t_height
            batch.begin(texture)
        },

        draw_tri(x: [number, number, number], y: [number, number, number], u: [number, number, number], v: [number, number, number]) {
            let xx = [x[0] / width, x[1] / width, x[2] / width]
            let yy = [y[0] / height, y[1] / height, y[2] / height]
            let uu = [u[0] / t_width, u[1] / t_width, u[2] / t_width]
            let vv = [v[0] / t_height, v[1] / t_height, v[2] / t_height]
            batch.draw_tri(xx, yy, uu, vv)
        },


        draw(x: number, y: number, w: number, h: number, sx: number, sy: number, flip_x: boolean) {
            let u = sx / t_width
            let v = sy / t_height
            let u2 = (sx + w) / t_width
            let v2 = (sy + h) / t_height

            x = Math.floor(x)
            y = Math.floor(y)

            x /= width
            y /= height
            w /= width
            h /= height

            if (flip_x) {
                [u, u2] = [u2, u]
            }

            batch.draw(x, y, w, h, u, v, u2, v2, [1, 1, 1, 1])
        },
        end_render() {
            batch.flush()
        },
        flush_to_screen() {
            gl.bindFramebuffer(gl.FRAMEBUFFER, null)

            gl.useProgram(screenShader)
            gl.viewport(0, 0, width, height)

            renderFullscreenQuad(gl, batch_render_target.texture, fullscreenQuadVAO)
        }
    }
}


function createRenderTarget(gl: WebGL2RenderingContext, width: number, height: number) {
  const texture = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, texture);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, width, height, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

  const fb = gl.createFramebuffer();
  gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
  gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, texture, 0);

  gl.bindFramebuffer(gl.FRAMEBUFFER, null)

  return { framebuffer: fb, texture: texture, width: width, height: height };
}

function renderFullscreenQuad(gl: WebGL2RenderingContext, texture: WebGLTexture, vao: WebGLVertexArrayObject) {
    gl.bindVertexArray(vao);
    
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, texture);
    
    gl.drawElements(
        gl.TRIANGLES,      // mode
        6,                // count (2 triangles × 3 vertices each)
        gl.UNSIGNED_SHORT, // type
        0                  // offset
    );
    
    gl.bindVertexArray(null);
}


function initFullscreenQuad(gl: WebGL2RenderingContext) {
    // Vertex positions (covering entire screen in clip space [-1,1])
    const vertices = new Float32Array([
        // Positions   // Texture coordinates
        -1.0, -1.0,    0.0, 0.0,  // Bottom-left
         1.0, -1.0,    1.0, 0.0,  // Bottom-right
         1.0,  1.0,    1.0, 1.0,  // Top-right
        -1.0,  1.0,    0.0, 1.0   // Top-left
    ]);
    
    // Triangle indices
    const indices = new Uint16Array([
        0, 1, 2,  // First triangle
        0, 2, 3   // Second triangle
    ]);
    
    // Create and bind VAO
    let fullscreenQuadVAO = gl.createVertexArray();
    gl.bindVertexArray(fullscreenQuadVAO);
    
    // Create and fill position + texture coordinate buffer
    let fullscreenQuadVBO = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, fullscreenQuadVBO);
    gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);
    
    // Create and fill element buffer
    let fullscreenQuadEBO = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, fullscreenQuadEBO);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, indices, gl.STATIC_DRAW);
    
    // Set up attribute pointers
    const FSQ_POSITION_LOCATION = 0;
    const FSQ_TEXCOORD_LOCATION = 1;
    
    // Position attribute
    gl.enableVertexAttribArray(FSQ_POSITION_LOCATION);
    gl.vertexAttribPointer(
        FSQ_POSITION_LOCATION,
        2,          // size (x,y)
        gl.FLOAT,   // type
        false,      // normalized
        4 * 4,      // stride (4 floats per vertex, 4 bytes per float)
        0           // offset
    );
    
    // Texture coordinate attribute
    gl.enableVertexAttribArray(FSQ_TEXCOORD_LOCATION);
    gl.vertexAttribPointer(
        FSQ_TEXCOORD_LOCATION,
        2,          // size (u,v)
        gl.FLOAT,   // type
        false,      // normalized
        4 * 4,      // stride
        2 * 4       // offset (skip first 2 floats)
    );
    
    // Unbind
    gl.bindVertexArray(null);
    gl.bindBuffer(gl.ARRAY_BUFFER, null);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, null);

    return fullscreenQuadVAO 
}
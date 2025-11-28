#version 300 es
precision mediump float;

layout(location = 0) in vec2 a_position;
layout(location = 1) in vec2 a_texCoord;
layout(location = 2) in vec4 a_color;

out vec2 v_texCoord;
out vec4 v_color;

void main() {
  v_texCoord = a_texCoord;
  v_color = a_color;

   // Alternative calculation that might be more precise
    vec2 pixelPos = a_position;
    vec2 clipPos = vec2(pixelPos.x * 2.0 - 1.0, pixelPos.y * -2.0 + 1.0);
    gl_Position = vec4(clipPos, 0.0, 1.0);
}
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
  gl_Position = vec4((a_position * 2.0 - 1.0) * vec2(1, -1), 0.0, 1.0);
}
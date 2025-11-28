precision mediump float;
uniform sampler2D uTexture;
uniform vec2 uTextureSize;  // Original render target size

varying vec2 vTexCoord;

void main() {
  // Convert to texture coordinates [0,1]
  vec2 texCoord = vTexCoord;
  
  // Apply zoom (centered zoom)
  texCoord = vec2(0.5) + (texCoord - vec2(0.5));
  
  
  // Sample texture (with optional pixel-perfect rounding)
  vec2 pixelCoord = texCoord * uTextureSize;
  // pixelCoord = floor(pixelCoord) + 0.5; // For crisp pixels
  texCoord = pixelCoord / uTextureSize;
  
  gl_FragColor = texture2D(uTexture, texCoord);
}
// Real manual 3 speed transmission --------------------------------------------------------------
// Curve for heavy trucks (low clutch engaging rpm)
float curveLinear[][2] = {{0, 0},     {83, 120},  {166, 196}, {250, 272},
                          {333, 348}, {416, 424}, {500, 500}, {600, 500}};

// ARRAY INTERPOLATION FUNCTION
// Credit: http://interface.khm.de/index.php/lab/interfaces-advanced/nonlinear-mapping/

uint32_t reMap(float pts[][2], uint32_t input) {
  uint32_t rr = 0;
  float mm = 0;

  for (uint8_t nn = 0; nn < 13; nn++) {
    if (input >= pts[nn][0] && input <= pts[nn + 1][0]) {
      mm = (pts[nn][1] - pts[nn + 1][1]) / (pts[nn][0] - pts[nn + 1][0]);
      mm = mm * (input - pts[nn][0]);
      mm = mm + pts[nn][1];
      rr = mm;
    }
  }
  return (rr);
}

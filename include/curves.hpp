// Real manual 3 speed transmission --------------------------------------------------------------
// Curve for heavy trucks (low clutch engaging rpm)
float curveLinear[][2] = {{0, 0}, {83, 120}, {166, 196}, {250, 272}, {333, 348}, {416, 424}, {500, 500}, {600, 500}};

// ARRAY INTERPOLATION FUNCTION
// Optimized version using binary search for O(log n) performance
// Original credit: http://interface.khm.de/index.php/lab/interfaces-advanced/nonlinear-mapping/
uint32_t reMap(float pts[][2], uint32_t input) {
  // Array size is known: 8 points (indices 0-7)
  constexpr uint8_t ARRAY_SIZE = 8;
  
  // Handle edge cases first
  if (input <= pts[0][0]) {
    return pts[0][1];
  }
  if (input >= pts[ARRAY_SIZE - 1][0]) {
    return pts[ARRAY_SIZE - 1][1];
  }
  
  // Binary search for the correct interval
  uint8_t left = 0;
  uint8_t right = ARRAY_SIZE - 2;  // Last valid interval index
  
  while (left <= right) {
    uint8_t mid = (left + right) / 2;
    
    if (input >= pts[mid][0] && input <= pts[mid + 1][0]) {
      // Found the interval - perform linear interpolation
      float x1 = pts[mid][0];
      float y1 = pts[mid][1];
      float x2 = pts[mid + 1][0];
      float y2 = pts[mid + 1][1];
      
      // Linear interpolation: y = y1 + (y2 - y1) * (x - x1) / (x2 - x1)
      float slope = (y2 - y1) / (x2 - x1);
      float result = y1 + slope * (input - x1);
      
      return (uint32_t)result;
    } else if (input < pts[mid][0]) {
      right = mid - 1;
    } else {
      left = mid + 1;
    }
  }
  
  // Fallback (shouldn't reach here with valid input)
  return pts[0][1];
}

// // vectors
// double v0_x = xC - xA;
// double v0_y = yC - yA;
// double v1_x = xB - xA;
// double v1_y = yB - yA;
// double v2_x = node->x - xA;
// double v2_y = node->y - yA;
//
// // dot products
// double dot00 = v0_x * v0_x + v0_y * v0_y;
// double dot01 = v0_x * v1_x + v0_y * v1_y;
// double dot02 = v0_x * v2_x + v0_y * v2_y;
// double dot01 = v1_x * v1_x + v1_y * v1_y;
// double dot02 = v1_x * v2_x + v1_y * v2_y;
//
// // barycentric coordinates
// double inv_den = 1 / (dot00 * dot11 - dot01 * dot01);
// double u = (dot11 * dot02 - dot01 * dot12) * inv_denom
// double v = (dot00 * dot12 - dot01 * dot02) * inv_denom
//
// // point is in triangle
// return (u >= 0) && (v >= 0) && (u + v < 1);

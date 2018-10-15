//This document includes the forward and reverse kinematics of a robot with 4 omni wheels
//Full explanation is here: ( https://docs.google.com/document/d/1IShP9O5Ke3e50hDjm0TVveUJd4YA53RzAsRMRkQrHEM/edit )
//For the first picture in the solution part

// Note : matrix of forward kinematics already takes into account 2 cylindrical transmissions (wheels 2 and 4)

float MLineSpeed[4][3] = { 24.4161859 ,   27.2213531 ,  0.0,  // forward linear matrix
                           31.09554175,  -15.04963672,  0.0,
                           31.07573411,   16.22788419,  0.0,
                           24.4161859,   -25.11085301,  0.0};


float MRotSpeed[4][3] = { 0.0,  0.0, -5.77869177,  // forward rotation matrix
                          0.0,  0.0,  4.47936896,
                          0.0,  0.0,  4.47936896,
                          0.0,  0.0, -5.77869177};


//For the third picture in the solution part
float InverseKinematics[3][4] = { 0.00735153, -0.01023156, -0.00975818,  0.00814357,  // inverse matrix
                                  0.01408402,  0.00840837, -0.00842103, -0.01407421,     
                                 -0.05287003, -0.04172438, -0.04279612, -0.05466322 };




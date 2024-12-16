void dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                         unsigned long const** col,
                                         unsigned long* nnz) {
   static unsigned long const rows[7] = {0,1,2,3,4,5,6};
   static unsigned long const cols[7] = {8,9,10,11,12,13,14};
   *row = rows;
   *col = cols;
   *nnz = 7;
}

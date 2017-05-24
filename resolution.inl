double resolution (pcl::PointCloud<pcl_point>::Ptr cloud_in, pcl::KdTreeFLANN<pcl_point>::Ptr tree)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);

  for (size_t i = 0; i < cloud_in->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud_in)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree->nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

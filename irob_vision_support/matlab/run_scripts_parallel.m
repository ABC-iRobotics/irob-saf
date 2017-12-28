parpool(2)
parfor K = 1 : 2
  if K == 1; vision_node_retraction; end
  if K == 2; RetractionControl; end
end
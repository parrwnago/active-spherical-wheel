# I-71 frozen evidence packets

The extractor reads only I70 model costs, validity and already selected
candidate IDs; prediction `time_s`; and original cached candidate points and
validity. It reads no contact truth, baseline family decisions, references,
terrain or future state. It calls no NNLS or geometry generator.

`stage01/CASE/inference_packet.npz` preserves all six original cost/ID/validity
arrays byte for byte. Its extra physical point maps are:

- `node_endpoint_points_m[N,21,3]`: the original singleton fit's first point
  for modes 0--7. Pair modes and unknown stay NaN, even if their old node fit was
  valid. An invalid original singleton fit also stays NaN.
- `edge_endpoint_points_m[N,6,2,2,3]`: for pair, current endpoint role and motion,
  select the original complete fit's point in that same endpoint role slot.
  This does not choose the closest or cheapest point independently.

All original full selected-ID arrays are retained. Endpoint-only IDs and masks
are supplied for audit. IDs are resolved through the copied original bank
layout's explicit candidate-ID-to-storage-slot map; integer IDs are not assumed
to be array offsets. Both points in every valid edge fit must belong to their
original families, be valid original candidates and be finite. No points are
averaged across bodies.

The consumer loads a case with `packet_io.load_packet(stage_root, case)`, which
verifies file hashes before returning eager NumPy arrays. The packet contains
no dependency on live simulator/controller modules.

Four synthetic contracts cover shuffled IDs/current-role selection, byte-exact
cost and ID preservation, unsupported/invalid endpoint NaNs, and rejection of
wrong-family, missing-ID or invalid-candidate fits. Source/input seals precede
the saved-data projection; metadata and artifact manifests bind each output.

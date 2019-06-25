defmodule ElixirRigidPhysics.Geometry.Capsule do
  require Record
  Record.defrecord(:capsule, axial_length: 0.5, cap_radius: 0.25)

  def create(axial_length, cap_radius),
    do: capsule(axial_length: axial_length, cap_radius: cap_radius)
end

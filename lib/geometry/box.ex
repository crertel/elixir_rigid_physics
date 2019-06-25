defmodule ElixirRigidPhysics.Geometry.Box do
  require Record
  Record.defrecord(:box, length: 0.0, width: 0.0, depth: 0.0)

  def create(l, w, d), do: box(length: l, width: w, depth: d)
end

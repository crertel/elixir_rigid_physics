defmodule ElixirRigidPhysics.Geometry.Sphere do
  require Record
  Record.defrecord(:sphere, radius: 1.0)

  def create(radius), do: sphere(radius: radius)
end

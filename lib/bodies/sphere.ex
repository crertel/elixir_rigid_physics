defmodule ElixirRigidPhysics.Bodies.Sphere do

  def make(r, mass, opts \\ []) do
    position = Keyword.get(opts, :position, {0,0,0})
    orientation = Keyword.get(opts, :rotate, {1,0,0,0})
    linear_dampening = Keyword.get(opts, :linear_dampening, 0)
    angular_dampening = Keyword.get(opts, :angular_dampening, 0)

    linear_velocity = Keyword.get(opts, :linear_velocity, {0,0,0})
    angular_velocity = Keyword.get(opts, :angular_velocity, {0,0,0})


    %{
      type: :sphere,
      radius: r,

      linear_dampening: linear_dampening,
      angular_dampening: angular_dampening,
      position: position,
      orientation: orientation,

      linear_velocity: linear_velocity,
      angular_velocity: angular_velocity,

      mass: mass,
      accumulated_force: {0,0,0},
      accumulated_torque: {0,0,0}
    }
  end
end

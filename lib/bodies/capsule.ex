defmodule ElixirRigidPhysics.Bodies.Capsule do

  def make(axial_length, end_radius, mass, opts \\ []) do
    position = Keyword.get(opts, :position, {0,0,0})
    orientation = Keyword.get(opts, :rotate, {1,0,0,0})
    linear_dampening = Keyword.get(opts, :linear_dampening, 0)
    angular_dampening = Keyword.get(opts, :angular_dampening, 0)

    linear_velocity = Keyword.get(opts, :linear_velocity, {0,0,0})
    angular_velocity = Keyword.get(opts, :angular_velocity, {0,0,0})


    %{
      type: :capsule,
      axial_length: axial_length,
      end_radius: end_radius,

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

defmodule ElixirRigidPhysics do
  @moduledoc """
  Documentation for ElixirRigidPhysics.
  """

  require Record

  Record.defrecord(:sim, world: nil, subscribers: MapSet.new(), next_tick: nil)

  use GenServer
  alias ElixirRigidPhysics.World

  #######################
  ## Client

  def start_link(%World{} = world) do
    GenServer.start_link(__MODULE__, sim(world: world))
  end

  def start_link(), do: start_link(%World{})

  def get_world_state(pid) do
    GenServer.call(pid, :get_world_state)
  end

  @doc """
  Steps the world simulation.

  Valid options are:

  * `timestep` is the number of seconds to step the world by.
  """
  @spec step_world(pid(), Keyword.t()) :: :ok
  def step_world(pid, opts \\ []) do
    GenServer.cast(pid, {:step_world, opts})
  end

  def add_body_to_world(pid, body) do
    GenServer.cast(pid, {:add_body_to_world, body})
  end

  def subscribe_to_world_updates(pid) do
    GenServer.cast(pid, {:subscribe_to_world_updates, self()})
  end

  def unsubscribe_from_world_updates(pid) do
    GenServer.cast(pid, {:unsubscribe_from_world_updates, self()})
  end

  def start_world_simulation(pid) do
    GenServer.cast(pid, :start_world_simulation)
  end

  def stop_world_simulation(pid) do
    GenServer.cast(pid, :stop_world_simulation)
  end

  #################################
  ## Server

  @impl true
  def init(s) do
    {:ok, s}
  end

  @impl true
  def handle_call(:get_world_state, _from, s) do
    world = sim(s, :world)
    {:reply, world, s}
  end

  @tick_rate 16

  @impl true
  def handle_cast({:step_world, opts}, s) do
    dt = Keyword.get(opts, :dt, @tick_rate / 1000)
    new_world = ElixirRigidPhysics.Dynamics.step(sim(s, :world), dt)

    update_subscribers(sim(s, :subscribers), new_world)

    {:noreply, sim(s, world: new_world)}
  end

  @impl true
  def handle_cast({:add_body_to_world, body}, s) do
    body_ref = make_ref()
    world = sim(s, :world)
    %World{bodies: bodies} = world
    new_world = %World{world | bodies: Map.put(bodies, body_ref, body)}

    update_subscribers(sim(s, :subscribers), new_world)

    {:noreply, sim(s, world: new_world)}
  end

  @impl true
  def handle_cast({:subscribe_to_world_updates, from_pid}, s) do
    subs = sim(s, :subscribers)

    {:noreply, sim(s, subscribers: MapSet.put(subs, from_pid))}
  end

  @impl true
  def handle_cast({:unsubscribe_from_world_updates, from_pid}, s) do
    subs = sim(s, :subscribers)

    {:noreply, sim(s, subscribers: MapSet.delete(subs, from_pid))}
  end

  @impl true
  def handle_cast(:start_world_simulation, s) do
    thandle = Process.send_after(self(), :tick_simulation, @tick_rate)
    {:noreply, sim(s, next_tick: thandle)}
  end

  @impl true
  def handle_cast(:stop_world_simulation, s) do
    thandle = sim(s, :next_tick)
    Process.cancel_timer(thandle)
    {:noreply, sim(s, next_tick: nil)}
  end

  @impl true
  def handle_info(:tick_simulation, s) do
    new_world = ElixirRigidPhysics.Dynamics.step(sim(s, :world), @tick_rate / 1000)
    thandle = Process.send_after(self(), :tick_simulation, @tick_rate)
    update_subscribers(sim(s, :subscribers), new_world)
    {:noreply, sim(s, world: new_world, next_tick: thandle)}
  end

  def update_subscribers(subscribers, world) do
    MapSet.to_list(subscribers)
    |> Enum.map(fn subscriber_pid ->
      send(subscriber_pid, {:world_update, world})
    end)
  end
end

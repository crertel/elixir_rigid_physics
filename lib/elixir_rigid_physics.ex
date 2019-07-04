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

  @spec start_link([{atom(), any()}]) :: {:error, any()} | {:ok, pid}
  def start_link(opts \\ []) do
    procname = Keyword.get(opts, :name, "")

    if procname !== "" do
      GenServer.start_link(__MODULE__, sim(world: %World{}), name: procname)
    else
      GenServer.start_link(__MODULE__, sim(world: %World{}))
    end
  end

  # @spec get_world_state(atom | pid | {atom, any} | {:via, atom, any}) :: any
  @spec get_world_state(atom | pid | {atom, any} | {:via, atom, any}) :: any
  def get_world_state(pid) do
    GenServer.call(pid, :get_world_state)
  end

  @doc """
  Steps the world simulation.

  Valid options are:

  * `timestep` is the number of seconds to step the world by.
  """
  @spec step_world(atom | pid | {atom, any} | {:via, atom, any}, [{atom, any}]) :: any
  def step_world(pid, opts \\ []) when is_list(opts) do
    GenServer.cast(pid, {:step_world, opts})
  end

  @spec add_body_to_world(atom | pid | {atom, any} | {:via, atom, any}, tuple) ::
          {:ok, reference()}
  def add_body_to_world(pid, body) when Record.is_record(body, :body) do
    body_ref = make_ref()
    {GenServer.cast(pid, {:add_body_to_world, body, body_ref}), body_ref}
  end

  @spec add_bodies_to_world(atom | pid | {atom, any} | {:via, atom, any}, [
          ElixirRigidPhysics.Dynamics.Body.body()
        ]) ::
          {:ok, [reference()]}
  def add_bodies_to_world(pid, bodies) when is_list(bodies) do
    body_refs = Enum.map(bodies, fn _ -> make_ref() end)
    {GenServer.cast(pid, {:add_bodies_to_world, Enum.zip(bodies, body_refs)}), body_refs}
  end

  @spec remove_body_from_world(atom | pid | {atom, any} | {:via, atom, any}, reference) :: :ok
  def remove_body_from_world(pid, body_ref) when is_reference(body_ref) do
    GenServer.cast(pid, {:remove_body_from_world, body_ref})
  end

  @spec remove_all_bodies_from_world(atom | pid | {atom, any} | {:via, atom, any}) :: :ok
  def remove_all_bodies_from_world(pid) do
    GenServer.cast(pid, :remove_all_bodies_from_world)
  end

  @spec subscribe_to_world_updates(atom | pid | {atom, any} | {:via, atom, any}) :: :ok
  def subscribe_to_world_updates(pid) do
    GenServer.cast(pid, {:subscribe_to_world_updates, self()})
  end

  @spec unsubscribe_from_world_updates(atom | pid | {atom, any} | {:via, atom, any}) :: :ok
  def unsubscribe_from_world_updates(pid) do
    GenServer.cast(pid, {:unsubscribe_from_world_updates, self()})
  end

  @spec start_world_simulation(atom | pid | {atom, any} | {:via, atom, any}) :: :ok
  def start_world_simulation(pid) do
    GenServer.cast(pid, :start_world_simulation)
  end

  @spec stop_world_simulation(atom | pid | {atom, any} | {:via, atom, any}) :: :ok
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

  # update at 60 hz
  # @tick_rate 16

  # update at 15 hz
  # @tick_rate 64

  # update at 30 hz
  @tick_rate 32

  @impl true
  def handle_cast({:step_world, opts}, s) do
    dt = Keyword.get(opts, :dt, @tick_rate / 1000)
    new_world = ElixirRigidPhysics.Dynamics.step(sim(s, :world), dt)

    update_subscribers(sim(s, :subscribers), new_world)

    {:noreply, sim(s, world: new_world)}
  end

  @impl true
  def handle_cast({:add_body_to_world, body, body_ref}, s) do
    %World{bodies: bodies} = world = sim(s, :world)
    new_world = %World{world | bodies: Map.put(bodies, body_ref, body)}

    update_subscribers(sim(s, :subscribers), new_world)

    {:noreply, sim(s, world: new_world)}
  end

  @impl true
  def handle_cast({:add_bodies_to_world, bodies}, s) do
    %World{bodies: old_bodies} = world = sim(s, :world)

    new_bodies =
      for {body, body_ref} <- bodies, into: old_bodies do
        {body_ref, body}
      end

    new_world = %World{world | bodies: new_bodies}

    update_subscribers(sim(s, :subscribers), new_world)

    {:noreply, sim(s, world: new_world)}
  end

  @impl true
  def handle_cast({:remove_body_from_world, body_ref}, s) do
    %World{bodies: bodies} = world = sim(s, :world)
    new_world = %World{world | bodies: Map.delete(bodies, body_ref)}

    update_subscribers(sim(s, :subscribers), new_world)

    {:noreply, sim(s, world: new_world)}
  end

  @impl true
  def handle_cast(:remove_all_bodies_from_world, s) do
    world = sim(s, :world)
    new_world = %World{world | bodies: %{}}

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

  @spec update_subscribers(MapSet.t(any), World.t()) :: :ok
  def update_subscribers(subscribers, world) do
    for subscriber_pid <- MapSet.to_list(subscribers),
        do: send(subscriber_pid, {:world_update, world})

    :ok
  end
end

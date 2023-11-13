#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use core::{mem::MaybeUninit, sync::atomic::{AtomicI32, Ordering}};
use embassy_executor::Executor;
use embassy_futures::select::select;
use embassy_time::{Timer, Duration};
use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay, IO, timer::TimerGroup, embassy, gpio::{Output, PushPull, Gpio4, Gpio3, PullUp, Input, Gpio5, Gpio2, Gpio1}};

use esp_wifi::{initialize, EspWifiInitFor};

use hal::{systimer::SystemTimer, Rng};
use rotary_encoder_hal::Rotary;
use static_cell::StaticCell;
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

static DELAY: AtomicI32 = AtomicI32::new(0);

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}


#[embassy_executor::task]
async fn motor(mut dir: Gpio1<Output<PushPull>>, mut step_pin: Gpio2<Output<PushPull>>) {
    loop {
        let delay = DELAY.load(Ordering::Relaxed);
        println!("Delay: {}",delay);

        if delay == 0 {
            Timer::after(Duration::from_millis(1)).await;
            continue;
        }
        if delay > 0 {
            dir.set_high().unwrap();
        } else {
            dir.set_low().unwrap();
        }
        let delay = delay.max(50).min(100_000);
        step_pin.toggle().unwrap();
        Timer::after(Duration::from_micros(delay.abs() as u64)).await
    }  
}

#[embassy_executor::task]
async fn encoder(pin_a: Gpio4<Input<PullUp>>,pin_b: Gpio5<Input<PullUp>>) {
    let mut rotary = Rotary::new(pin_a, pin_b);
    let mut count = 0_i32;
    loop {
        let (pin_a,pin_b) = rotary.pins();
        select(pin_a.wait_for_any_edge(),pin_b.wait_for_any_edge()).await;
        let direction = rotary.update().unwrap();
        match direction {
            rotary_encoder_hal::Direction::Clockwise => count+=1,
            rotary_encoder_hal::Direction::CounterClockwise => count-=1,
            rotary_encoder_hal::Direction::None => (),
        }
        println!("Count: {}",count);
        if count != 0 {
            DELAY.store(1_000_000 / count.pow(3), Ordering::Relaxed)
        } else {
            DELAY.store(0, Ordering::Relaxed);
        }

    }
}

#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    // let mut delay = Delay::new(&clocks);

    static EXECUTOR: StaticCell<Executor> = StaticCell::new();

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    println!("Hello world!");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);
    let pin1 = io.pins.gpio1.into_push_pull_output();
    let pin2 = io.pins.gpio2.into_push_pull_output(); 
    let pin3 = io.pins.gpio3.into_push_pull_output();
    let pin4 = io.pins.gpio4.into_pull_up_input();
    let pin5 = io.pins.gpio5.into_pull_up_input();

    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());
    
    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks, &mut system.peripheral_clock_control);    
    embassy::init(&clocks,timer_group.timer0);

    executor.run(|spawner| {
        spawner.spawn(encoder(pin4, pin5)).unwrap();
        spawner.spawn(motor(pin1, pin2)).unwrap();

    });


    // let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    // let _init = initialize(
    //     EspWifiInitFor::Wifi,
    //     timer,
    //     Rng::new(peripherals.RNG),
    //     system.radio_clock_control,
    //     &clocks,
    // )
    // .unwrap();




}

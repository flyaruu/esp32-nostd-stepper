#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Executor;
use embassy_futures::select::select;
use embassy_time::{Timer, Duration};
use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay, IO, timer::TimerGroup, embassy, gpio::{Output, PushPull, Gpio4, Gpio3, PullUp, Input, Gpio5}};

use esp_wifi::{initialize, EspWifiInitFor};

use hal::{systimer::SystemTimer, Rng};
use rotary_encoder_hal::Rotary;
use static_cell::StaticCell;
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[embassy_executor::task]
async fn encoder(pin_a: Gpio4<Input<PullUp>>,pin_b: Gpio5<Input<PullUp>>) {
    let mut rotary = Rotary::new(pin_a, pin_b);
    let mut count = 0_i64;
    loop {
        let (pin_a,pin_b) = rotary.pins();
        select(pin_a.wait_for_any_edge(),pin_b.wait_for_any_edge()).await;
        let direction = rotary.update().unwrap();
        match direction {
            rotary_encoder_hal::Direction::Clockwise => count+=1,
            rotary_encoder_hal::Direction::CounterClockwise => count-=1,
            rotary_encoder_hal::Direction::None => (),
        }
        println!("Count: {}",count)
    }
}

#[embassy_executor::task]
async fn blink_red(mut pin: Gpio3<Output<PushPull>>) {
    loop {
        println!("Loop...");
        pin.toggle().unwrap();
        // delay.delay_ms(500u32);
        Timer::after(Duration::from_millis(330)).await;
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
    let pin3 = io.pins.gpio3.into_push_pull_output();
    let pin4 = io.pins.gpio4.into_pull_up_input();
    let pin5 = io.pins.gpio5.into_pull_up_input();

    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());
    
    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks, &mut system.peripheral_clock_control);    
    embassy::init(&clocks,timer_group.timer0);

    executor.run(|spawner| {
        spawner.spawn(blink_red(pin3)).unwrap();
        spawner.spawn(encoder(pin4, pin5)).unwrap();


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

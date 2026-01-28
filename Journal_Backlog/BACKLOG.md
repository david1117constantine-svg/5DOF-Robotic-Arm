# Journal Backlog: October 24th, 2025 - January 8th 2026

WORK IN PROGRESS


------------
##12/30/2025: Ive done it once, I'll do it again... Redesign time!

The initial Upper arm design I've been visualizing in my head is simply far too heavy to even stand a chance of realistically working. Im now going for something almost opposite, a skeleton design. The arm is going to look bare bones, but it will be lighter, maybe even stronger, and pretty easy to prototype with. Plus, saving filament is always a bonus. Ive designed a new piece that fits in the shoulder and consists of two disks connected with 4, strut-like pillars keeping them together. The pillars have holes running all the way through and can be screwed together with M3x25 screws and nuts. After I confirmed all the dimensions with a test print, I added an interface to build the Upper Arm's frame off of. Here's what it looks like in CAD: 

<img width="570" height="455" alt="image" src="https://github.com/user-attachments/assets/c08c84c7-513f-41f3-9ed1-267ea1a7e2b3" />

## 12/24/2025: Its Christmas Eve, and the first iteration of the Upper Arm mount to the shoulder. 
Prototype? The goal is to have everything internalized so the outside looks super clean. I ran into a ton of issues with jamming the entire motor assembly into the casing. It still doesn't totally fit, but I put the newest version together,r and I can definitely see the vision. 

![IMG_5965](https://github.com/user-attachments/assets/ff6eb031-101c-4c6c-a996-b71b06e71767)

I have been using a little script I wrote that can use serial commands to control a single motor, and the shoulder does move, but with much difficulty. Weight seems like it will be an issue, but until then, that is where I'm at.

## 12/22/2025: Holidays are ramping up, and so am I.
Finally printed the last of the shoulder pieces. Took a few iterations; the first time, I didn't even include all the holes for the screws; too excited, I guess.

![IMG_5961](https://github.com/user-attachments/assets/820593d5-d95d-4d83-a7c4-306750447f88)

![IMG_5962](https://github.com/user-attachments/assets/f39a5fa4-76f9-4ee7-b8bf-e42cf67c6a7d)

Forgot to mention in the last entry that the gear-like hole in the motor side of the shoulder is for the output shaft of a simple planetary gearbox design I found on printables. *(Future me wants to add I did not go with this design, but it's a combination of ProfessorBoots' 4:1 planetary gearboxes and @flolorenzo's 28BYJ48 motor adapter)* One 4:1 gearbox is not enough, according to my calculations, but thankfully, this design is stackable, so I went with two gearboxes, totalling a 16:1 reduction.

![IMG_5964](https://github.com/user-attachments/assets/95e06255-b95d-4a5e-8d6c-fdb7e1bf3999)

## 12/15/2025: Shouler redesign went quite painlessly; lets hope I didnt just jinx anything!
The new design features what I outlined in the previous entry: removable shoulder arms to allow for easier assembly, one simple loft instead of some really jacked-up ones like in the previous design, and now one side has a groove for a toothed shaft to fit into, allowing the arm to actually turn.

<img width="727" height="647" alt="image" src="https://github.com/user-attachments/assets/bbda72e5-2955-48ee-8fe0-4824179ada1e" />

## 12/11/2025: CAD sucks, and I'm restarting the shoulder. (Lofts are the bane of my existence)

Been working on the shoulder the past couple of days, finally got something that I was kind of proud of. 

<img width="687" height="630" alt="image" src="https://github.com/user-attachments/assets/7fda78cb-fedd-4a89-86b9-ab32019979d2" />

Honestly, it doesn't look that bad; it isn't that bad, except for a few obvious flaws. First, you will notice that both sides of the shoulder mount have cuts for bearings to sit in. If there are bearings on either side, how is the Upper arm supposed to move? The second major flaw is with the physical construction of the shoulder joint. I think I am going to start over and move towards a design in which I can remove either side of the Shoulder mount arm thingies to aid in assembly. When both sides are fixed, you would basically have to bend them in order to squeeze something into the middle, unless you ran some sort of rod inbetween but I don't think I'm gonna do that.

## 12/4/2025: New Printer!
Just got a new 3d printer that's gonna help in printing all the larger pieces of the arm! It's a Creality Ender 3 S1, and it was broken when a friend gave it to me, but it only took about 20 minutes to fix the software issues, just reflashed the firmware, and it whirred to life.

![IMG_5928](https://github.com/user-attachments/assets/dc3f90d4-281e-4155-b77c-097ac1190c30)

![IMG_5929](https://github.com/user-attachments/assets/320b8169-9af6-42f4-bec1-6ec44e589c5b)

## 12/1/2025: The Waist Continues!!!
Started printing the next couple of pieces. The inside of the waist is closed off with a type of cap, which also has a tray on top for the bearing to sit in and be screwed into place. 

<img width="1279" height="918" alt="Screenshot 2026-01-18 104253" src="https://github.com/user-attachments/assets/09f5554c-cf89-43a5-a229-68a065264809" />

<img width="1064" height="686" alt="image" src="https://github.com/user-attachments/assets/504fc172-d46d-43a6-a143-6da41dfd1137" />

## 11/30/2025: Locking in; Waist progressing smoothly
I decided to start building the arm around these cheap stepper motors I found in my school's computer lab, specifically the 28-BYJ-48 with ULN2003 drivers. They seem pretty cheap, so that is good for my budget, which is basically nothing. Added the ribbing on the side for effect, looks cool, I think, and maybe it provides marginal support theoretically. Using little holes i measured out that you can pop nuts into, and then you get threads, cheaper and more accessible than heatset inserts; also less permanent. The only hardware I have right now is what is left from the 3D printer build. It won't last me the entire build, but for now, it will have to do.

![IMG_5874](https://github.com/user-attachments/assets/5023fef2-9b05-4af8-bf71-7b1adc66b085)

<img width="597" height="497" alt="Screenshot 2026-01-13 204241" src="https://github.com/user-attachments/assets/0c2e7fa6-87f4-42a2-9c12-2a728f01b68c" />

You may be able to see a little bit of material that sticks out around the screw holes in the motor mount. This began by accident; however turned out that it works in my favor. The mounting holes on the 28BYJ-48 steppers are for M4 screws; however, the only screws I have on hand are M3. The M3 screws thread into the plastic, which in turn squeezes the raised material outward, centering the motor over the screw holes, and squeezing them tight onto the posts. Cool right?

<img width="586" height="493" alt="Screenshot 2026-01-13 204302" src="https://github.com/user-attachments/assets/400df47f-18f7-4cb0-907b-289c91c2aaaf" />

Cool design feature I should mention. For now, I am sticking with the extrusions for a simple solution to a base/mount for the arm. I designed the waist with the idea of multiple mounting options in mind. There is an X pattern of crews and a circular depression in the bottom of the waist, allowing it to be connected to many mounting solutions. Still brainstorming how the actual waist joint is going to stay in place, planning on buying bearings of some sort for the spinning aspect, but we will see, I guess.

Here it is on the printer, complete, I might add.

![IMG_5877](https://github.com/user-attachments/assets/265ff6ee-b27a-4ab8-9124-c74bf45df07a)

## 11/29/2025: Tuning, Tuning, Tuning; Ready to print 1st part of the Arm!
Designed a base piece that would connect the extrusions and bolt onto whatever the design for the Waise Joint will be. The first print came out fine, needed some clearance fixes though, went back into cad and it's all good now!

<img width="720" height="581" alt="Screenshot 2026-01-13 201932" src="https://github.com/user-attachments/assets/d06d4c69-3b51-4816-9e31-4e34df3ded53" />

<img width="749" height="540" alt="Screenshot 2026-01-13 201914" src="https://github.com/user-attachments/assets/8af8f9f3-3b1d-488d-ac7b-e46399c1afd9" />

![IMG_5870](https://github.com/user-attachments/assets/384c8ee4-fe45-43a4-b8f6-7d58d994b5cd)
This was the 1st version, which was missing drilled holes for connecting the waist, along with too loose clearances.

![IMG_5875](https://github.com/user-attachments/assets/743355f7-d7a5-44bc-9cb3-a670040033ab)
V2 on the Voron

## 11/16/2025: Voron is done, first print is out of the way; Back to CAD!
Did some thinking about my previous design, and I think I'm gonna scratch the entire thing and start over. Gonna use 2020 extrusions along, and start from there.

<img width="844" height="595" alt="Screenshot 2026-01-13 201837" src="https://github.com/user-attachments/assets/9dcf42f1-aa3e-4af7-af3b-762c23881338" />

## 10/25/2025: Last of the Voron parts came; Project on standby for now
Not much to this entry. I'm super excited to finally have a 3D printer again. I need to find a place to keep all the different benchys I'm about to make!

## 10/24/2025: The Birth of an Idea
Yesterday I got my new Voron V0.2 in the mail! Decided to start modeling the glimpse of an idea I saw in my head of a Robotic Arm, built all with hobby-grade 3D printing. Looks pretty trash right now, but I think with some work it should be ok. I've got a lot to learn about Fusion 360 still, so this is probably good for me. using 4040 extrusions for the base, hopefully that is a wide and sturdy enough structure to hold the arm up. Thats all I've got for now!

<img width="422" height="322" alt="IMG_5698" src="https://github.com/user-attachments/assets/a05fb64b-f419-4af4-a55f-ecc1ed1d0d35" />

P.S. Sorry for the terrible quality image; it was a screenshot from my phone.

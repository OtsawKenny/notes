Persona 1 — Landlord/Agent “Adrian”

Motivations
- Fill the room/flat quickly
- Minimize time wasted chatting with bad candidates
- Avoid unsafe or unreliable tenants

Behaviors
- Posts listings frequently
- Manages multiple rooms or properties
- Responds fast to serious inquiries

Persona 2 — Tenant “Kim” (Looking For Room)

Motivations
- Find an affordable room fast
- Prefer safe, reliable landlords
- Don’t want to waste time viewing bad rooms
- Want convenience + trust
Behaviors
- Scrolls many listings
- Filters by price, MRT, room type
- Contacts multiple landlords at once
- Wants fast replies

Persona 3 — Roommate Seeker “Ray” (Looking For Housemates)
Motivations
- Find someone compatible to rent together
- Avoid bad experiences (unclean, loud, weird habits)
- Wants transparency

Behaviors
- Browses profiles
- Reads lifestyle details
- Messages people with aligned habits
- Wants early filtering (“no smoker”, “pet friendly”)


⭐ Landlord Goals
Get a good tenant fast
Minimize bad applicants / time wasters
Showcase the room in the best light
Receive and respond to inquiries quickly
Feel trusted & legitimate to tenants

⭐ Tenant Goals (Searching Rooms)
- Find a suitable room that meets needs & budget
- Avoid scams / bad landlords
- Contact landlords quickly
- See clear, honest info upfront
- Shortlist easily, compare easily

⭐ Roommate Seeker Goals
- Find compatible roommates
- Trust the person before meeting
- Avoid awkward or unsafe interactions
- Share lifestyle expectations upfront
- Message safely and anonymously before exchanging contact info



🔹 Landlord Pain Points
Gets too many unqualified or incompatible inquiries
Tenants ghost / no-show
Listing takes too long to create
No way to prove legitimacy or trustworthiness
Hard to manage multiple listings
What design must solve
Optimize listing creation (fast, minimal)
Provide tenant filtering / pre-profile
Give verification badges
Give a good messaging inbox for landlords


🔹 Tenant Pain Points
Listings often lack real details
Hard to know “hidden rules” (cooking allowed? AC? Visitors?)
Agents post fake listings
Takes too many steps to contact
Hard to compare multiple listings
Hard to filter by MRT/area effectively
They fear scams
What design must solve
Clear listing structure
Required fields (AC, wifi, cooking, gender pref, min stay)
Verified landlord flag
Simple message CTA
Smart search filters (price, area, MRT, room type)
Saved/favorited listings
Photos first

🔹 Roommate Seeker Pain Points
Finding compatible lifestyles is hard
People hide big lifestyle factors (noise, pets, smoking)
No way to safely message someone without giving phone number
Too much friction to compare roommate profiles
What design must solve
Rich roommate profiles (habits, cleanliness, daily schedule)
Compatibility indicators
Anonymous chat
Lifestyle filters
Status: “Actively looking / Not looking”


Example (Tenant):
- Browsing rooms
- Filtering rooms
- Reading details
- Contacting landlords
- Saving favorites

Example (Landlord):
- Creating listings
- Managing listings
- Responding to tenants

Exmaple (Tenant - Roommate Seeker Activities)
Creating a roommate profile
Browsing roommate profiles
Filtering for compatible roommates
Reading lifestyle & preference details
Contacting potential roommates
Saving / bookmarking profiles
Chatting to build trust
Arranging meetups or viewings


1️⃣ Actors (who’s in the story)
- Landlord
- Tenant / Room Seeker
- Roomies System (backend)
- Payment Provider (Stripe/Momo/VNPay etc., later)
- Admin / Moderator

2️⃣ Big Picture Domains
Think of these as mini-systems:
- Identity & Profiles
- Listings & Availability
- Search & Matching
- Messaging & Contact
- Trust & Verification
- Boost & Monetization

Contracts & Agreements

Moderation & Reporting

We’ll event-storm a few main flows across these.

3️⃣ Flow 1: Landlord creates & manages a room listing
Timeline (Domain Events)

UserRegistered

ProfileCompleted

LandlordRoleActivated (user chooses “I’m a landlord” / “I have a room to rent”)

ListingDraftCreated

ListingPhotosUploaded

ListingDetailsUpdated (price, address, room type, rules, etc.)

ListingSubmittedForReview

ListingApproved (by automated checks or admin)

ListingPublished

ListingBoostPurchased (optional)

ListingBoostActivated

ListingBoostExpired

ListingUpdated (price changed, title updated, etc.)

ListingSuspended (policy violation or landlord pause)

ListingUnpublished (room rented or landlord removes it)

Commands that trigger those events

RegisterUser

CompleteProfile

ActivateLandlordRole

CreateListingDraft

UploadListingPhotos

UpdateListingDetails

SubmitListingForReview

ApproveListing

PublishListing

PurchaseListingBoost

ActivateListingBoost

ExpireListingBoost

UpdateListing

SuspendListing

UnpublishListing

Main Aggregates here

User (id, profile, roles)

Listing (id, landlordId, status, details, photos, boost state)

Boost (id, listingId, type, startAt, endAt)

4️⃣ Flow 2: Tenant searches & contacts landlord
Timeline (Domain Events)

TenantSearchStarted

SearchFiltersApplied (city, budget, duration, gender preference, etc.)

ListingsViewed

ListingFavorited

ContactRequestSent (tenant writes message / click “Contact landlord”)

ConversationThreadCreated

MessageSent

MessageDelivered

MessageRead

ViewingRequested (tenant wants to view)

ViewingScheduled

ViewingCompleted

TenantShortlistedByLandlord

TenantRejectedByLandlord

TenantConfirmedInterest

RoomReservedForTenant (soft lock / tentative)

ReservationExpired (tenant no response or time limit reached)

Commands

StartTenantSearch

ApplySearchFilters

ViewListing

FavoriteListing

SendContactRequest

SendMessage

ScheduleViewing

ConfirmViewing

MarkViewingCompleted

ShortlistTenant

RejectTenant

ConfirmTenantInterest

ReserveRoomForTenant

ExpireReservation

Aggregates

SearchSession (ephemeral, maybe not a true aggregate, but you can track)

Conversation (id, participants, messages)

Viewing (id, listingId, tenantId, time, status)

Reservation (id, listingId, tenantId, expiresAt)




1. MVP SCOPE (ONLY WHAT IS NEEDED)
Your MVP should have 3 things only:

✔ 1. Landlord can post a room
Title
Description
Price
Location
Room type
Photos
Gender preference
Contact (phone or “chat”)

✔ 2. Tenant can browse & filter
City
District
Price
Gender preference
Room type
Verified landlord badge (optional)

✔ 3. Tenant → Landlord contact
Simple chat OR redirect to landlord’s Zalo/phone
(keep it super simple first)
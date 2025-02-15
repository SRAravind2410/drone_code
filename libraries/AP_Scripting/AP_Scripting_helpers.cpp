#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "AP_Scripting_helpers.h"
#include <AP_Scripting/lua_generated_bindings.h>

/// Fast param access via pointer helper class

// Custom lua constructor with optional param name
int lua_new_Parameter(lua_State *L) {

    const int args = lua_gettop(L);
    if (args > 1) {
        return luaL_argerror(L, args, "too many arguments");
    }
    const char * name = nullptr;
    if (args == 1) {
        name = luaL_checkstring(L, 1);
    }

    // This chunk is the same as the auto generated constructor
    void *ud = lua_newuserdata(L, sizeof(Parameter));
    new (ud) Parameter();
    luaL_getmetatable(L, "Parameter");
    lua_setmetatable(L, -2);

    if (args == 0) {
        // no arguments, nothing to do
        return 1;
    }

    if (!static_cast<Parameter*>(ud)->init(name)) {
        return luaL_error(L, "No parameter: %s", name);
    }

    return 1;
}

// init by name
bool Parameter::init(const char *name)
{
    vp = AP_Param::find(name, &vtype);
    if (vp == nullptr) {
        return false;
    }
    return true;
}

// init by info, to get the value of old params
bool Parameter::init_by_info(uint16_t key, uint32_t group_element, enum ap_var_type type)
{
    switch (type) {
    case AP_PARAM_INT8:
        vp = NEW_NOTHROW AP_Int8;
        break;
    case AP_PARAM_INT16:
        vp = NEW_NOTHROW AP_Int16;
        break;
    case AP_PARAM_INT32:
        vp = NEW_NOTHROW AP_Int32;
        break;
    case AP_PARAM_FLOAT:
        vp = NEW_NOTHROW AP_Float;
        break;
    default:
        return false;
    }
    if (vp == nullptr) {
        return false;
    }
    vtype = type;
    AP_Param::ConversionInfo info = {
        key,
        group_element,
        type,
        nullptr
    };
    return AP_Param::find_old_parameter(&info, vp);
}

// set a value
bool Parameter::set(float value)
{
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

// get value
bool Parameter::get(float &value)
{
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        value = ((AP_Int8 *)vp)->get();
        break;
    case AP_PARAM_INT16:
        value = ((AP_Int16 *)vp)->get();
        break;

    case AP_PARAM_INT32:
        value = ((AP_Int32 *)vp)->get();
        break;

    case AP_PARAM_FLOAT:
        value = ((AP_Float *)vp)->get();
        break;

    default:
        // not a supported type
        return false;
    }
    return true;
}

// set and save value
bool Parameter::set_and_save(float value)
{
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set_and_save(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

// Check if param had been configured
bool Parameter::configured()
{
    if (vp == nullptr) {
        return false;
    }
    return vp->configured();
}

// set default value
bool Parameter::set_default(float value)
{
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set_default(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set_default(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set_default(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set_default(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

#if HAL_ENABLE_DRONECAN_DRIVERS

#define IFACE_ALL uint8_t(((1U<<(HAL_NUM_CAN_IFACES))-1U))

/************************************************
  DroneCAN broadcast and request message handling
 ************************************************/

/*
  broadcast a message, takes a lua string as payload
 */
int DroneCAN_Handle::broadcast(lua_State *L)
{
    binding_argcheck(L, 2);

    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);

    size_t data_length;
    const void *data = lua_tolstring(L, 2, &data_length);
    
    auto &iface = h->dc->get_canard_iface();
    Canard::Transfer transfer;

    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.data_type_signature = h->signature;
    transfer.data_type_id = h->data_type;
    transfer.inout_transfer_id = &h->transfer_id;
    transfer.priority = 0;
    transfer.payload = data;
    transfer.payload_len = data_length;
    transfer.iface_mask = IFACE_ALL;
    transfer.canfd = h->canfd;
    transfer.timeout_ms = 10;

    bool ok = iface.broadcast(transfer);
    lua_pushboolean(L, ok);

    return 1;
}

/*
  send a request message that expects a reply
 */
int DroneCAN_Handle::request(lua_State *L)
{
    binding_argcheck(L, 3);

    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);
    uint8_t target_node = luaL_checknumber(L, 2);

    size_t data_length;
    const void *data = lua_tolstring(L, 3, &data_length);

    auto &iface = h->dc->get_canard_iface();

    Canard::Transfer transfer;

    transfer.transfer_type = CanardTransferTypeRequest;
    transfer.data_type_signature = h->signature;
    transfer.data_type_id = h->data_type;
    transfer.inout_transfer_id = &h->transfer_id;
    transfer.priority = 0;
    transfer.payload = data;
    transfer.payload_len = data_length;
    transfer.iface_mask = IFACE_ALL;
    transfer.canfd = h->canfd;
    transfer.timeout_ms = 10;

    auto *hsem = Canard::HandlerList::get_semaphore(h->dc->get_driver_index());
    if (hsem == nullptr) {
        return 0;
    }
    WITH_SEMAPHORE(*hsem);

    if (h->subscriber != nullptr) {
        delete h->subscriber;
    }
    
    h->subscriber = NEW_NOTHROW Subscriber(*h, CanardTransferTypeResponse);
    bool ok = h->subscriber != nullptr;

    if (ok) {
        h->subscriber->node_id = target_node;
        ok &= iface.request(target_node, transfer);
    }

    lua_pushboolean(L, ok);

    return 1;
}

/*
  subscribe to broadcast messages
 */
bool DroneCAN_Handle::subscribe(void)
{
    auto *hsem = Canard::HandlerList::get_semaphore(dc->get_driver_index());
    if (hsem == nullptr) {
        return false;
    }
    WITH_SEMAPHORE(*hsem);
    if (subscriber != nullptr) {
        delete subscriber;
    }
    subscriber = NEW_NOTHROW Subscriber(*this, CanardTransferTypeBroadcast);
    return subscriber != nullptr;
}

DroneCAN_Handle::Subscriber::Subscriber(DroneCAN_Handle &_handle, CanardTransferType _transfer_type) :
    Canard::HandlerList(_transfer_type, _handle.data_type, _handle.signature, _handle.dc->get_driver_index())
{
    handle = &_handle;
    trans_type = _transfer_type;
    WITH_SEMAPHORE(sem[index]);
    link();

    // link it into cleanup list
    cleanup_next = cleanup;
    if (cleanup != nullptr) {
        cleanup->cleanup_prev = this;
    }
    cleanup = this;
}

DroneCAN_Handle::Subscriber::~Subscriber(void)
{
    WITH_SEMAPHORE(sem[index]);
    unlink();
    for (uint8_t i=0; i<MAX_PAYLOADS; i++) {
        free(payloads[i]);
        payloads[i] = nullptr;
    }

    // remove from cleanup list
    if (cleanup_next) {
        cleanup_next->cleanup_prev = cleanup_prev;
    }
    if (cleanup_prev) {
        cleanup_prev->cleanup_next = cleanup_next;
    }
    if (this == cleanup) {
        cleanup = cleanup_next;
    }
}

/*
  handle an incoming subscribed message
 */
bool DroneCAN_Handle::Subscriber::handle_message(const CanardRxTransfer& transfer)
{
    WITH_SEMAPHORE(handle->sem);

    uint8_t pidx = 0;
    while (pidx < MAX_PAYLOADS && payloads[pidx] != nullptr) {
        pidx++;
    }
    if (pidx == MAX_PAYLOADS) {
        // already have MAX_PAYLOADS messages pending, discard this one
        return false;
    }

    if (transfer.transfer_type == CanardTransferTypeResponse &&
        (node_id != transfer.source_node_id ||
         ((transfer.transfer_id+1)&0xFF) != handle->transfer_id)) {
        // not from right node, or not right transfer ID
        return false;
    }

    uint8_t *buffer = (uint8_t *)malloc(transfer.payload_len);
    if (buffer == nullptr) {
        // discard messages if we can't allocate
        return false;
    }

    node_id = transfer.source_node_id;
#if CANARD_ENABLE_CANFD
    handle->canfd = transfer.canfd;
#endif
    uint32_t bits_remaining = transfer.payload_len * 8;
    uint32_t ofs = 0;

    // decode in bytes, we don't have a canardDecodeBuffer function
    while (bits_remaining > 0) {
        uint8_t nbits = MIN(8U, bits_remaining);
        canardDecodeScalar(&transfer, ofs*8, nbits, false, (void*)&buffer[ofs]);
        ofs++;
        bits_remaining -= nbits;
    }

    // fill in the payload
    payloads[pidx] = buffer;
    payload_lengths[pidx] = transfer.payload_len;

    return true;
}

/*
  check for an incoming message
 */
int DroneCAN_Handle::check_message(lua_State *L)
{
    /*
      get mutex on the HandlerList for this driver then the
      DroneCAN_Handle mutex. Note that order is important
     */
    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);
    auto *hsem = Canard::HandlerList::get_semaphore(h->dc->get_driver_index());
    if (hsem == nullptr) {
        return 0;
    }
    WITH_SEMAPHORE(*hsem);
    WITH_SEMAPHORE(h->sem);

    auto *s = h->subscriber;

    if (s == nullptr || s->payloads[0] == nullptr) {
        return 0;
    }

    lua_pushlstring(L, (char *)s->payloads[0], s->payload_lengths[0]);
    lua_pushinteger(L, s->node_id);

    // remove the first payload and shift others up
    // for small MAX_PAYLOADS this is pretty efficient
    free(s->payloads[0]);
    s->payload_lengths[0] = 0;

    memmove(&s->payloads[0], &s->payloads[1], sizeof(s->payloads[0])*(MAX_PAYLOADS-1));
    memmove(&s->payload_lengths[0], &s->payload_lengths[1], sizeof(s->payload_lengths[0])*(MAX_PAYLOADS-1));
    s->payloads[MAX_PAYLOADS-1] = nullptr;
    s->payload_lengths[MAX_PAYLOADS-1] = 0;

    if (s->trans_type == CanardTransferTypeResponse) {
        // request reply removes the subscriber
        delete h->subscriber;
        h->subscriber = nullptr;
    }
    
    return 2;
}

/*
  cleanup any dangling subscribers
 */
void DroneCAN_Handle::destroy_all(void)
{
    /*
      we need to stop any processing from happening while we cleanup
     */
    for (uint8_t i=0; i<CANARD_NUM_HANDLERS; i++) {
        auto *h = Canard::HandlerList::get_semaphore(i);
        if (h != nullptr) {
            h->take_blocking();
        }
    }
    while (Subscriber::cleanup) {
        delete Subscriber::cleanup;
    }
    for (uint8_t i=0; i<CANARD_NUM_HANDLERS; i++) {
        auto *h = Canard::HandlerList::get_semaphore(i);
        if (h != nullptr) {
            h->give();
        }
    }
}

/*
  garbage collect a handle
 */
int DroneCAN_Handle::__gc(lua_State *L)
{
    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);
    auto *hsem = Canard::HandlerList::get_semaphore(h->dc->get_driver_index());
    if (hsem == nullptr) {
        return 0;
    }
    WITH_SEMAPHORE(*hsem);
    WITH_SEMAPHORE(h->sem);

    if (h->subscriber != nullptr) {
        delete h->subscriber;
        h->subscriber = nullptr;
    }
    return 0;
}

// lua constructor for DroneCAN_Handle
int DroneCAN_Handle::new_handle(lua_State *L)
{
    static bool metatable_created;
    if (!metatable_created) {
        metatable_created = true;
        luaL_newmetatable(L, "DroneCAN_Handle");
    
        lua_pushcfunction(L, DroneCAN_Handle::__gc);
        lua_setfield(L, -2, "__gc");
    
        lua_pop(L, 1);
    }

    lua_Number bus_index = luaL_checknumber(L, 1);

    auto *dc = AP_DroneCAN::get_dronecan(bus_index);
    if (dc == nullptr) {
        return luaL_error(L, "Invalid DroneCAN bus: %d", int(bus_index));
    }

    // This chunk is the same as the auto generated constructor
    void *ud = lua_newuserdata(L, sizeof(DroneCAN_Handle));
    new (ud) DroneCAN_Handle();

    auto *h = static_cast<DroneCAN_Handle*>(ud);

    h->dc = dc;

    luaL_getmetatable(L, "DroneCAN_Handle");
    lua_setmetatable(L, -2);

    return 1;
}

HAL_Semaphore DroneCAN_Handle::sem;
DroneCAN_Handle::Subscriber *DroneCAN_Handle::Subscriber::cleanup;

#endif // HAL_ENABLE_DRONECAN_DRIVERS

#endif  // AP_SCRIPTING_ENABLED

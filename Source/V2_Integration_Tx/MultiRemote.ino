// V2.5-Evo - 2026-10-20 - MULTI-REMOTE REMOTE STEP (TX SW29): new file. The remote's side of several remotes on one
//   buggy (design v3.1 section 9, v3.2 sections 3.4.2 and 5) that is not radio code and not a gesture: the boot
//   identity line (design 9.13). The radio side (frames, status, rate, pairing) stays in Radio.ino, the gestures in
//   Hall.ino / RTMState.ino. Every function here runs on the loop task.

// ============================================================
// THE BOOT IDENTITY LINE (design 9.13, MT-00)
// ============================================================
// mtxTxBootReport - one "MTX:" line after the config is loaded: this chip's ID (eFuse), the stored own_address, and the
//   link: new-style (proto 3: the buggy's ID and the link ID L this remote sends), old-style (proto 0: the buggy's
//   address, today's frames), or not paired. Two NOTEs where they apply: a stored own_address that is not this chip's
//   ID (a copied config - harmless on a new-style link, where the ID comes from the chip), and no_lock, which also
//   turns off the spare-remote auto-lock. Inputs: g_efuse_id, usrConf. Side effects: Serial only.
//   (Whether this remote is the master is the buggy's answer - SEIZABLE - so ?mtx shows it, not this line.)
void mtxTxBootReport()
{
  char me[9], own[9], dest[9];
  snprintf(me,   sizeof(me),   "%02X:%02X:%02X", g_efuse_id[0], g_efuse_id[1], g_efuse_id[2]);
  snprintf(own,  sizeof(own),  "%02X:%02X:%02X", usrConf.own_address[0], usrConf.own_address[1], usrConf.own_address[2]);
  snprintf(dest, sizeof(dest), "%02X:%02X:%02X", usrConf.dest_address[0], usrConf.dest_address[1], usrConf.dest_address[2]);
  if (!usrConf.paired)
  {
    Serial.printf("MTX: id %s (eFuse), own_address %s, not paired - hold RIGHT at power-on to pair\n", me, own);
  }
  else if (txIsV3())
  {
    MtxLinkRec r;
    txLinkRec(&r);
    uint8_t L[3];
    mtxUplinkAddr(&r, L);
    Serial.printf("MTX: id %s (eFuse), own_address %s, link v3 to %s, L %02X:%02X:%02X\n", me, own, dest, L[0], L[1], L[2]);
  }
  else
  {
    Serial.printf("MTX: id %s (eFuse), own_address %s, link legacy to %s (old-style frames, as before; pair again for the multi-remote link)\n",
                  me, own, dest);
  }
  if (!mtxIdEqual(usrConf.own_address, g_efuse_id))
    Serial.printf("NOTE: stored own_address %s differs from this board's eFuse id %s (config copied?) - a new-style link uses the chip ID\n", own, me);
  if (usrConf.no_lock && txIsV3())
    Serial.println("NOTE: no_lock is set - a spare remote will not lock itself");
}
